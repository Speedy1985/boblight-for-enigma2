/*
 * grabber-enigma2
 * Copyright (C) 2014 Martijn Vos(Speedy1985) and Oktay Oeztueter <wp.oktay.com>
 
 * parts of this code were taken from
 *
 * - aiograb		(http://schwerkraft.elitedvb.net/projects/aio-grab/)
 * - boblight-X11	
 * 
 * boblight is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * boblight is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "util/inclstdint.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#define BOBLIGHT_DLOPEN
#include "util/misc.h"
#include "util/timeutils.h"
#include "util/lock.h"
#include "util/log.h"

#include "lib/boblight.h"
#include "grabber-enigma2.h"

//Rgb Tables
#include "include/rgbtables.inc"

using namespace std;

volatile bool m_stop = false;

CGrabber::CGrabber(void* boblight, volatile bool& stop, bool sync) : m_stop(stop), m_timer(&stop)
{   
    // The handler
	m_boblight      = boblight;

	//Set grab values
	chr_luma_stride           = 0x40;
	chr_luma_register_offset  = 0;
	registeroffset            = 0;
	mem2memdma_register       = 0;

	// Set defaults
	m_cluster       = 1;        // Default
	blank           = false;    // Default
	m_debug         = true;     // Default
	m_sync          = true;     // Default
	m_grabinfo      = false;    // Default
	m_interval      = 0.1f;     // Default interval is 10ms
	m_last_beam_process = 0.0;  //
	m_last_res_process  = 0.0;  //

	luma            = NULL;     // Set to NULL
	chroma          = NULL;	    // Set to NULL

	xres_old        = 0;
	yres_old        = 0;
	xres_tmp        = 0;
	yres_tmp        = 0;
	skiplines       = 2;
	m_mode          = 2;        // Default dynamic
	m_brightness    = 255;      // Default brightness for fader
	m_3d_mode       = 1;        // Normal mode
	m_delay         = 0;
	m_lightsoff     = false;   

	if(m_debug)
	{
	    //Set fps counters
	    fps_lastupdate  = GetTimeUs();
	    fps_framecount  =0;
	    m_lastupdate    = GetTimeSec<long double>();
	    m_lastmeasurement = m_lastupdate;
	    m_measurements  = 0.0;
	    m_nrmeasurements = 0.0;
	}		

	//Set our logfile
    logtostderr = true;
    SetLogFile("boblight-enigma2.log");
	     
}

CGrabber::~CGrabber()
{
    Log("Free all memory...\n");
    
	// free all memory
	if (luma)
		free(luma);
	if (chroma)
		free(chroma);
    if (video)
		free(video);

	// close handle on memory
	close(mem_fd);
	
	Log("Delete socket file...\n");
	
	// delete socket file  
    unlink(NAME);
	
	//StopThreads();
	exit(0);
}

bool CGrabber::Setup()
{
    //Get number off lights    
    nrlights = boblight_getnrlights(m_boblight);
    int channels = nrlights*3;
   
	// set up for the timer
	if (m_interval > 0.0)
	{
        m_timer.SetInterval(Round64(m_interval * 1000000.0));
	}

	// Clear errors
	m_error.clear();
		
    // Detect stb
	if (!detectSTB()) { //If unknown then return.
		LogError("detect STB failed!");
		return false; //Stop boblight
	}
	else
	{
        Log("Detected STB-Type: brcm%d",stb_type);
        if(m_debug){
            Log("Stb settings: chr_luma_stride %x", chr_luma_stride);
            Log("Stb settings: chr_luma_register_offset %x", chr_luma_register_offset);
            Log("Stb settings: registeroffset %x", registeroffset);
            Log("Stb settings: mem2memdma_register %x", mem2memdma_register);
        }
        
        Log("Try to open memory...");                        
		mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
		if (mem_fd < 0) {
		    LogError("Can't open memory....");
			return false;
		}
		
	}
    
	Log("Lights: %d, Cluster leds: %d as one led",nrlights,m_cluster);
	Log("Channels: %d",channels);
	Log("Interval: %.2f",m_interval);
	
	if(m_3d_mode != 1)
    	Log("3D mode: %i",m_3d_mode);
	
	if (m_debug) 
		Log("Debug mode: enabled");
	if (m_sync) 
		Log("Sync mode: enabled");
    if (m_blackbar) 
		Log("Blackbar mode: enabled");
    
    if(m_mode == 1)
        Log("Mode: Static");
    else if(m_mode == 2)
        Log("Mode: Dynamic");
    else if(m_mode == 3)
        Log("Mode: RGB Test");
    else if(m_mode == 4)
        Log("Mode: Color fader");                
    	
	luma   = (unsigned char*)malloc(1920*300*3);
    chroma = (unsigned char*)malloc(1920*300*3/2);
     
    //prepare the video-buffer (vastly oversized!!)
	video  = (unsigned char*)malloc(300*300*4);
	memset(video,0,300*300*4);
	
	// Set some vars to default values
	bb_toggle = give_error = false;
	skip_blackYlines,blackYLines=bbv_=bbv__=bb_black=bb_color=bb_lines=bb_lines_=adjust_x=adjust_y=m_fps=bb_old=0;
	filename_count=0;

	return true; // All ok? then return true and start Run();
}

bool CGrabber::Run()
{
    int i,yStart;
    int m_brightness_old = m_brightness;
    int rgb[3] = {m_brightness , m_brightness , m_brightness};
    int newRGB[3] = {m_brightness , m_brightness , m_brightness};

    float factor = 0.0;
    float actualBrightness;
    
    startThreads();  // Start socket loop
    
    while(!m_stop)
	{
        adjust_x = 0;	    
	
	    /////////////////
        //
        // Dyamic mode
        //
        
	    if(m_mode == 2)
	    {    
	    
	        // Grab the video        
	        if (!grabVideo())
                if(blank==false)
			        continue;		       			          

		    // Set new xres.	    
            if(xres > 2 and yres > 2)
                adjust_x = xres;                       
	
	
	        // Check resolution/3dmode
            if  (m_old_3d_mode != m_3d_mode || (yres_old!=yres) || (xres_old!=xres) || yres <= 1 || yres >= yres_orig/2 || xres <= 1 || xres >= xres_orig/2) 
            {  
	            blank=true;

	            //Reset some globals
	            bb_toggle=false;skip_blackYlines=0,blackYLines=0;
			      
	            if(m_old_3d_mode != m_3d_mode || xres > 2 && yres > 2)
	            {            
                    yres_old=yres; xres_old=xres;
		            setscanRange(adjust_x,yres);
	            }
            }

	        // Check and Send
            if(blank || yres <= 1 || xres <= 1 || adjust_x == 0){

                //If set_black is false then send once black to lights
                if(!set_black){
                    
                    // Set black
                    set_black = true;
                    int black[3] = {0, 0 ,0};
                        
                    // Send black frame to boblight
                    //boblight_addpixel(m_boblight, -1, black);
                                        
                    // Reset video
                    memset(video,0,300*300*4);
                    boblight_addbitmap(m_boblight, &video[0], xres, yres, m_delay);  //send bitmap to libboblight, there it will filter al the values  
                    if (!boblight_sendrgb(m_boblight,1, NULL,m_cluster))
    	                PrintError(boblight_geterror(m_boblight));
    	            
    	            yres_tmp=xres_tmp=0;
    	            
                    if(m_debug)
                        Log("[Debug] Nothing to grab, Lights off");
                    
                }else{
                    usleep(50000);
                }

            }
            else{
              
                // Set black to false
                set_black  = false;
                
                // Set to false so we can receive new errors.
                give_error = false;
                
                /// Check for blackbars every second
                if (m_blackbar){
                    
                    long double now = GetTimeSec<long double>();
                    
                    if (now - m_last_beam_process >= 1.0){
                        m_last_beam_process = now;
                        beamProcess();
                    }
                    
                    //yres = yres - (skip_blackYlines*2);        
                    yStart = skip_blackYlines;
                    
                }else{

                    yStart = 0;
                    yres  = yres;
                }

	            // Convert to rgb format and apply start and end
	            CovertVideo(yStart);
	                
	            // Picdump if enabled
                if (m_picdump){SaveBMP(xres,yres,3);}
                
		    	// Send complete videobitmap to libraryfile
	            boblight_addbitmap(m_boblight, &video[0], xres, yres, m_delay);  //send bitmap to libboblight, there it will filter al the values                
	            
	            // Now send all values from libboblight to leds.
                if (!boblight_sendrgb(m_boblight,1, NULL,m_cluster)) 
    	            PrintError(boblight_geterror(m_boblight));	            
            }
            
            //Wait some ms                           
            m_timer.Wait(); 

    	    //Update fps counter
	        UpdateDebugFps();

        }        
        else if(m_mode == 1)    //Static mode
        {
            //load the color into int array
            int rgb[3] = {(m_color >> 16) & 0xFF, (m_color >> 8) & 0xFF, m_color & 0xFF};
   
            //set all lights to the color we want and send it
            boblight_addpixel(m_boblight, -1, rgb);
            
            boblight_sendrgb(m_boblight,1, NULL,m_cluster);
            
            //sleep some ms
            usleep(50000);
        }
        else if(m_mode == 3)    //Test RGB
        {            
            usleep(50000);
            
            int red[3]   = {255, 0, 0};
            int green[3] = {0, 255, 0};
            int blue[3]  = {0, 0, 255};
            
            printf("\nMode: Test RGB:\n");
            
            printf("  red..\n");
            boblight_addpixel(m_boblight, -1, red);
            boblight_sendrgb(m_boblight,1, NULL,m_cluster);
            sleep(1);
            
            printf("  green..\n");
            boblight_addpixel(m_boblight, -1, green);
            boblight_sendrgb(m_boblight,1, NULL,m_cluster);
            sleep(1);
            
            printf("  blue..\n");
            boblight_addpixel(m_boblight, -1, blue);
            boblight_sendrgb(m_boblight,1, NULL,m_cluster);
            sleep(1);
            
            printf("\nExit..\n");
            m_stop = true;

        }
        else if(m_mode == 4) // Moodlamp
        {
            if(m_brightness != m_brightness_old)
            {
                 for (int a=0; a < 3; a++)
                 {
                    rgb[a] = m_brightness;
                    newRGB[a] = m_brightness;
                 } 
                 
                 m_brightness_old = m_brightness;
            }
            
	        while((!m_stop) && (m_mode == 4) && (rgb[0] != newRGB[0] || rgb[1] != newRGB[1] || rgb[2] != newRGB[2]))
	        {

		        for(i =0; i < 3; i++)
		        {
			        if(rgb[i] < newRGB[i])
			        {
				        rgb[i]++;
			        } else if(rgb[i] > newRGB[i])
			        {
				        rgb[i]--;
			        }
		        }		

		        boblight_addpixel(m_boblight, -1, rgb);

		        if (!boblight_sendrgb(m_boblight,1, NULL,m_cluster))
    	            PrintError(boblight_geterror(m_boblight));
    	        
		        usleep(100 * 1000);

	        }
	        newRGB[0] = rand() % 0xFF;       
	        newRGB[1] = rand() % 0xFF;       
	        newRGB[2] = rand() % 0xFF;       

	        actualBrightness = sqrt(newRGB[0] * newRGB[0] * 0.299 + newRGB[1] * newRGB[1] * 0.578 + newRGB[2] * newRGB[2] * 0.114);
	        factor = m_brightness / actualBrightness;
	        newRGB[0] *= factor;
	        newRGB[1] *= factor;
	        newRGB[2] *= factor;	

	        newRGB[0] =  MIN(0xFF, newRGB[0]);
	        newRGB[1] =  MIN(0xFF, newRGB[1]);
	        newRGB[2] =  MIN(0xFF, newRGB[2]);
        }
        else if(m_mode == 5){ //Rainbow

	        int rgb[7][3] = {{0xFF, 0, 0}, {0xFF, 0x80, 0}, {0xFF, 0xFF,  0}, {0, 0xFF, 0}, {0, 0, 0xFF}, {0x4B, 0, 0x82}, {0xEE, 0x82, 0xEE}};
            int numPix = boblight_getnrlights(m_boblight);
	        int  finalRGB[numPix][3];
	        int i, j;

	        for(i = 0; i < numPix; i++)
	        {
		        finalRGB[i][0] = rgb[7 * i/numPix][0];
		        finalRGB[i][1] = rgb[7 * i/numPix][1];
		        finalRGB[i][2] = rgb[7 * i/numPix][2];
	        }

	        for(i = 0; i < numPix; i++)
	        {
			        boblight_addpixel(m_boblight, i, finalRGB[numPix - i]);
	        }
	        
	        if (!boblight_sendrgb(m_boblight,1, NULL,m_cluster))
    	            PrintError(boblight_geterror(m_boblight));
    	        
		    usleep(20 * 1000); 
	   }
	}
	
}

void CGrabber::UpdateDebugFps()
{
	long double now = GetTimeSec<long double>(); 		// current timestamp
	m_measurements += now - m_lastmeasurement;			// diff between last time we were here
	m_nrmeasurements++;									// got another measurement
	m_lastmeasurement = now;							// save the timestamp
	
	if (now - m_lastupdate >= 1.0)						// if we've measured for one second, place fps on ouput.
	{
	      m_lastupdate = now;

	      if (m_nrmeasurements > 0) m_fps = 1.0 / (m_measurements / m_nrmeasurements); // we need at least one measurement
	      m_measurements = 0.0;
	      m_nrmeasurements = 0;
          
          //write fps and res. info to boblight directory, GUI can read from that file.
          writeFPS();                    
          
	      if(m_debug)
	      {                
                if(!blank){
		             Log("[Debug] gFPS:%2.1f | Skip:%d | Res:%dx%d (%dx%d)",m_fps,skiplines,xres,yres,xres_orig,yres_orig);
		             if(m_blackbar)
    		             Log("[Debug:blackbarinfo] bbv_=%i bbv__=%i skip_blackYlines=%d,black_lines=%d",bbv_,bbv__,skip_blackYlines,(bb_lines+bb_lines_)>>1);
		        }else{
		             Log("[Debug] gFPS:%2.1f | No video input... xres:%d yres:%d",m_fps,xres_orig,yres_orig);		             
		        }
	      }
	}
}


void CGrabber::setscanRange(int xres, int yres)
{
    if(xres !=xres_orig && yres != yres_orig){
        if(m_3d_mode == 1)
        {
            boblight_setscanrange(m_boblight, xres, yres); //normal
            Log("Set Scanrange to %dx%d (Source %dx%d)",xres,yres,xres_orig,yres_orig);
        }
        else if(m_3d_mode == 2)
        {
            boblight_setscanrange(m_boblight, xres, yres/2); //topandbottom
            Log("Set Scanrange to %dx%d (Source %dx%d)",xres,yres/2,xres_orig,yres_orig/2);
            Log("3D Mode: TAB");        
        }
        else if(m_3d_mode == 3)
        {
            boblight_setscanrange(m_boblight, xres/2, yres); //sidebyside
            Log("Set Scanrange to %dx%d (Source %dx%d)",xres/2,yres,xres_orig/2,yres_orig);
            Log("3D Mode: SBS");
        }    
    }

    //Saves the 3dmode to check every loop for changes
    m_old_3d_mode = m_3d_mode;
        
}             

int CGrabber::checkResolution(int stride, int *xres, int *yres, long double now)
{
    ///Check for resolution every 10 seconds or if xres/yres is not ok    
    if (now - m_last_res_process >= 10.0 || m_last_res_process <= 0.0 || stride != xres_tmp || yres_tmp <= 0 || xres_tmp <= 0){
        m_last_res_process = now;
        
	    // get resolutions from the proc filesystem
	    yres_tmp = hexFromFile("/proc/stb/vmpeg/0/yres");
	    xres_tmp = hexFromFile("/proc/stb/vmpeg/0/xres");
    }
    
    *xres = xres_tmp;
    *yres = yres_tmp;
}

bool CGrabber::grabVideo()
{
    blank = false;
    int stride = 0;
    unsigned char* memory_tmp;
    				
	//grab pic from decoder memory
	const unsigned char* data = (unsigned char*)mmap(0, 100, PROT_READ, MAP_SHARED, mem_fd, registeroffset);
   
    if(data == MAP_FAILED){
	  if(give_error != true)
        LogError("Mainmemory data: <Memmapping failed>");
	  return false;
	}

    unsigned int adr,adr2,ofs,ofs2,offset,pageoffset,counter=0;
    
    // Wait till we get a sync from the videodecoder.
    while (1) {
        unsigned int val = ((volatile unsigned int*)data)[0x30/4];
        if (val & 1){
            nice(0);
            break;
        }        

        usleep(1000);
        counter++;
        
        if(counter > 50)
            break;
    }    

	ofs = data[chr_luma_register_offset + 8] << 4;      /* luma lines */
	ofs2 = data[chr_luma_register_offset + 12] << 4;    /* chroma lines */	
	adr = (data[0x1f] << 24 | data[0x1e] << 16 | data[0x1d] << 8); /* start of videomem */
	adr2 = (data[chr_luma_register_offset + 3] << 24 | data[chr_luma_register_offset + 2] << 16 | data[chr_luma_register_offset + 1] << 8);	
	stride = data[0x15] << 8 | data[0x14];
	
	checkResolution(stride, &xres,&yres,GetTimeSec<long double>());	
    
	offset=adr2-adr;
	
    pageoffset = adr & 0xfff;
    adr -= pageoffset;
    adr2 -= pageoffset;
	
	//munmap(data, 100);
	munmap((void*)data, 100);

    // Check that obtained values are sane and prevent segfaults.
    if (!adr || !adr2 || (adr2 <= adr) || yres <= 0 || xres <= 0)
    {        
        //we need this give_error bool, others we get a loop of 1000 errors in some min.
        if(give_error != true){
            if(m_debug)
                LogError("Got invalid memory offsets, retry... (adr=0x%x,adr2=0x%x)", adr, adr2);             
            give_error = true;
        }
        xres=yres=xres_orig=yres_orig=0;blank = true;
        return false;
	}			
	else if (stride < xres/2)
	{	    
	    if(give_error != true){
	        if(m_debug)
                LogError("X-Resolution != stride: %d",stride);
            give_error = true;
        }
		blank = true;return false;
	}
	 
	int memory_tmp_size = 0;
	
	//Set xres
	xres = stride;
	
	// store original resolution
	xres_orig=xres; yres_orig=yres;
	
    m_size=128;
    skiplines = 2;
    
    //Check wich resolution is higher
    int skipres = yres;
    if(xres > yres)
        skipres = xres;
        
    while(skipres/skiplines > m_size){
        skiplines *= 2;
    }
 
	if (!mem2memdma_register)
	{ 	    
		// on dm800/dm500hd we have direct access to the decoder memory
		memory_tmp_size = offset + (stride + chr_luma_stride) * ofs2;

		memory_tmp = (unsigned char*)mmap(0, memory_tmp_size, PROT_READ, MAP_SHARED, mem_fd, adr);
        
		if (memory_tmp == MAP_FAILED || memory_tmp == NULL) {
		    
		    if(give_error != true){
		        LogError("Mainmemory: <Memmapping failed>");
		        give_error = true;
		    }
			blank = true;return false;
		}

	}
	else
	{
        int tmp_size  = offset + (stride + chr_luma_stride) * ofs2;
        
		if (tmp_size > 2 * DMA_BLOCKSIZE)
		{
		    if(give_error != true){
                LogError("Got invalid stride value from the decoder: %d", stride);
			    give_error = true;
			} 
			blank=true;return false;
		}
		
		memory_tmp_size = DMA_BLOCKSIZE + 0x1000;
		
		memory_tmp = (unsigned char*)mmap(0, DMA_BLOCKSIZE + 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, SPARE_RAM);
		volatile unsigned long *mem_dma;
		
		if(!(mem_dma = (volatile unsigned long*)mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, mem2memdma_register)))
		{
		    if(give_error != true){
                LogError("Mainmemory: <Memmapping failed>");
                give_error = true;
            }    
			blank=true;return false;
		}

		int i = 0;
		int tmp_len = DMA_BLOCKSIZE;
		
		for (i=0; i < tmp_size; i += DMA_BLOCKSIZE)
		{
			unsigned long *descriptor = (unsigned long*)memory_tmp;

			if (i + DMA_BLOCKSIZE > tmp_size)
				tmp_len = tmp_size - i;
			
	        if(m_grabinfo)
    			Log("DMACopy: %x (%d) size: %d\n", adr+i, i, tmp_len);
			
			descriptor[0] = /* READ */ adr + i;
			descriptor[1] = /* WRITE */ SPARE_RAM + 0x1000;
			descriptor[2] = 0x40000000 | /* LEN */ tmp_len;
			descriptor[3] = 0;
			descriptor[4] = 0;
			descriptor[5] = 0;
			descriptor[6] = 0;
			descriptor[7] = 0;
			mem_dma[1] = /* FIRST_DESCRIPTOR */ SPARE_RAM;
			mem_dma[3] = /* DMA WAKE CTRL */ 3;
			mem_dma[2] = 1;
			while (mem_dma[5] == 1)
				usleep(2);
			mem_dma[2] = 0;
		}
	    
		munmap((void *)mem_dma, 0x1000);
		
		/* unmap the dma descriptor page, we won't need it anymore */
		munmap((void *)memory_tmp, 0x1000);
		/* adjust start and size of the remaining memory_tmp mmap */
		memory_tmp += 0x1000;
		memory_tmp_size -= 0x1000;
	}

	// Some extra debug info
	if(m_grabinfo)
	    printf("\n[Grabinfo] X-Res: %i Y-Res: %i FPS:%2.1f Adr: %X Adr2: %X OFS,OFS2: %d %d = %d C-offset:%d\n",stride,yres,m_fps,adr,adr2,ofs,ofs2,ofs+ofs2,offset);
    
    
	//
	// decode luma & chroma plane or lets say sort it
	//
	unsigned int x,y,luna_mem_pos = 0, chroma_mem_pos = 0, dat1 = 0;         
    
    int blocksize   = chr_luma_stride;
    int skip        = chr_luma_stride*skiplines; // Skip mem position
	int skipx	= skiplines;
	
	//Fix for some strange resolution, from Oktay
	if((stride/2)%2==1)
		stride-=2;

	for (x = 0; x < stride; x += chr_luma_stride)
    {
        // check if we can still copy a complete block.
        if ((stride - x) <= chr_luma_stride)
            blocksize = stride-x;

        dat1 = x;    // 1088    16 (68 x)
        for (y = 0; y < ofs; y+=skiplines)
        {
            int z1=0;
            int skipofs=0;
            for(int y1=0;y1<blocksize;y1+=skipx)
            {
                *(luma + (dat1/skipx)+(y1/skipx))=*(y1+memory_tmp + pageoffset + luna_mem_pos);

                if (y < ofs2 && z1%2==0)
                {
                    skipofs=1;
                    chroma[(dat1/skipx)+(y1/skipx)]=*(y1+memory_tmp + pageoffset + offset + chroma_mem_pos);
                    chroma[(dat1/skipx)+(y1/skipx)+1]=*(1+y1+memory_tmp + pageoffset + offset + chroma_mem_pos);     
                }
                z1++;
            }

            if(skipofs==1)
                chroma_mem_pos += skip;
                
            skipofs=0;
            dat1 += stride;
            luna_mem_pos += skip;
                      
        }

        //Skipping invisble lines
        if ( (xres == 1280 && yres == 1080) ) luna_mem_pos += (ofs - yres) * chr_luma_stride;
    }
    
    //Set new resolution
    yres = yres/skiplines;
    xres = xres/skiplines;

	if (yres%2 == 1)
		yres--;		// drop one line to make the numer even again
    
                           
	// un-map memory
	munmap(memory_tmp, memory_tmp_size);
        
	return true;
}

bool CGrabber::CovertVideo(int yStart)
{
    int  x,y,posb;					                // loop counters
	long pos_chr, pos_luma, pos_rgb, rgbskip;	// relative position in chroma- and luma-map
	int  Y, U, V, RU, GU, GV, BV;			    // YUV-Coefficients
	int  U2, V2, RU2, GU2, GV2, BV2;		    // YUV-Coefficients
	
	if (yStart%2 == 1)
		yStart--;		// drop one number to make the numer even again    
	
	// yuv2rgb conversion (4:2:0)
	rgbskip = xres * 3;
	
	//setscanRange(xres ,yres);
	
	pos_rgb = 0;

    //y is start vanaf waar kleur begint	
	for (y=yStart; y < yres; y+=2) //laten doorlopen tot normale yres/skiplines
	{	    
		for (x=0; x < xres; x+=2)
		{
            pos_luma = x + (y * xres);
			pos_chr  = x + (y * xres / 2);
			
			// chroma contains both U and V data
			
			U2=chroma[pos_chr+1];	//2
			V2=chroma[pos_chr+0];	//3 litte->big endian :)
 		
			RU2=yuv2rgbtable_ru[V2]; 
			GU2=yuv2rgbtable_gu[U2];
			GV2=yuv2rgbtable_gv[V2];
			BV2=yuv2rgbtable_bv[U2];

			//If picdump then swap, because bmp use BGR
			if(m_picdump)
			{
			    SWAP(RU,BV);
			    SWAP(RU2,BV2);
			}			
		
			// now we do 4*2 pixels on each iteration this is more code but much faster 
 			Y=yuv2rgbtable_y[luma[pos_luma]]; 
			video[pos_rgb+0]=CLAMP((Y + RU2)>>16);
			video[pos_rgb+1]=CLAMP((Y - GV2 - GU2)>>16);
			video[pos_rgb+2]=CLAMP((Y + BV2)>>16);
 			
			Y=yuv2rgbtable_y[luma[xres+pos_luma]];
			video[pos_rgb+0+rgbskip]=CLAMP((Y + RU2)>>16);
			video[pos_rgb+1+rgbskip]=CLAMP((Y - GV2 - GU2)>>16);
			video[pos_rgb+2+rgbskip]=CLAMP((Y + BV2)>>16);
			 			
			pos_rgb  +=3;	

			Y=yuv2rgbtable_y[luma[pos_luma+1]];
			video[pos_rgb+0]=CLAMP((Y + RU2)>>16);
			video[pos_rgb+1]=CLAMP((Y - GV2 - GU2)>>16);
			video[pos_rgb+2]=CLAMP((Y + BV2)>>16);

			Y=yuv2rgbtable_y[luma[xres+pos_luma+1]];
			video[pos_rgb+0+rgbskip]=CLAMP((Y + RU2)>>16);
			video[pos_rgb+1+rgbskip]=CLAMP((Y - GV2 - GU2)>>16);
			video[pos_rgb+2+rgbskip]=CLAMP((Y + BV2)>>16);

			pos_rgb  +=3;	// skip forward for the next group of 4 pixels
		}
		pos_rgb+=rgbskip;	// skip a complete line
	}
	
	yres = yres - (skip_blackYlines*2);
}	

bool CGrabber::SaveBMP(int xres_, int yres_, int output_bytes) {
	
	// XXX Todo Bitmaps are BGR, my Video Data is RGB
	
	char buffer [50]; // stores the filename
	sprintf (buffer, "/media/hdd/tmp/boblight_enigma2_%04d.bmp", filename_count++);
	
	FILE *fd2 = fopen(buffer, "wr");
	if (!fd2) {
		perror(buffer);
		return 1;
	}

	// write header
	unsigned char hdr[14 + 40];
	unsigned int i = 0;
	// BMP-Header
	PUT8('B'); PUT8('M');				// header
	PUT32((((xres * yres) * 3 + 3) &~ 3) + 14 + 40);		// file size
	PUT16(0); PUT16(0); 				// reserved
	PUT32(14 + 40);						// start of pixel data
	// DIB HEADER
	PUT32(40); 							// Header Size
	PUT32(xres); 						// xres
	PUT32(yres);						// yres
	PUT16(1);							// number of color planes
	PUT16(24); 							// bits-per-pixel
	PUT32(0);	                        // BI_RGB, no pixel array compression used
	PUT32(0); 	                        // Size of the raw data in the pixel array (including padding)
	PUT32(0); 	                        // Horizontal resolution of the image
	PUT32(0); 	                        // Vertical resolution of the image
	PUT32(0); 	                        // Number of colors in the palette
	PUT32(0); 	                        // 0 means all colors are important @0x32
/*	PUT32(255); 		                // red  channel mask
	PUT32(255<<8); 		                // green channel mask
	PUT32(255<<16); 	                // blue channel mask
	PUT32(0);		                    // alpha channel mask
	PUT32(0x57696E20);	                // LCS_WINDOWS_COLOR_SPACE */
	//more bytes which i dont care for :)
	fwrite(hdr, 1, i, fd2);
    
    Log("[Picdump] Saving image to %s (%d,%d) xres:%d yres:%d",buffer,xres_,yres_,xres,yres);
      
	// write data
	int y; 
	int bmp_tmp=0;
	 
	int null=0;
	 
	for (y=yres-1; y>=0 ; y-=1) {
		
		bmp_tmp=0;
		fwrite(video+(y*xres*output_bytes),xres*output_bytes,1,fd2);
		while(  (xres*output_bytes+bmp_tmp++)%4!=0)
			fwrite(&null,1,1,fd2);
			
	}
	fclose(fd2);
	//printf("... Done writing the file\n");
	
}

/*
  Helper Functions
*/

int CGrabber::hexFromFile(const char *filename)
{	
	FILE* fd = fopen(filename, "r");
	if (!fd) return -1;
	int result = -1;
	fscanf(fd, "%x", &result);
	fclose(fd);
	return result;
}

const char *CGrabber::file_getline(const char *filename)
{
	static char *line = NULL;
	static size_t n = 0;
	ssize_t ret;
	FILE *f;

	f = fopen(filename, "r");
	if (f == NULL) {
		perror(filename);
		return NULL;
	}

	ret = getline(&line, &n, f);

	fclose(f);

	if (ret < 0)
		return NULL;

	while (ret-- > 0) {
		if ((line[ret] != '\n') &&
			(line[ret] != '\r'))
			break;
		line[ret] = '\0';
	}

	return line;
}


int CGrabber::file_scanf_line(const char *filename, const char *fmt, ...)
{
	const char *line = file_getline(filename);
	va_list ap;
	int ret;

	if (line == NULL)
		return -1;

	va_start(ap, fmt);
	ret = vsscanf(line, fmt, ap);
	va_end(ap);

	return ret;
}


void CGrabber::writeFPS(){    
  ofstream file;
  
  file.open ("/home/boblight-addons/bob_fps");
  
  char buffer [256];
  int n;
  sprintf(buffer, "%2.1f fps @ %dx%d(%dx%d)", m_fps,xres,yres,xres_orig,yres_orig);

  file << buffer;
  file.close();
}


//Socket connection for GUI
void CGrabber::socketProcess()
{
      int sock, msgsock, rval;
      struct sockaddr_un server;
      char buf[1024];
      
      // delete socket file  
      unlink(NAME);

      sock = socket(AF_UNIX, SOCK_STREAM, 0);
      if (sock < 0) {
          perror("opening stream socket");
          exit(1);
      }
      server.sun_family = AF_UNIX;
      strcpy(server.sun_path, NAME);
      if (bind(sock, (struct sockaddr *) &server, sizeof(struct sockaddr_un))) {
          perror("binding stream socket");
          exit(1);
      }
      Log("Socket has name %s", server.sun_path);
      listen(sock, 5);
      for (;;) {
          msgsock = accept(sock, 0, 0);
          if (msgsock == -1){
              perror("accept");
          }
          else do 
          {
                bzero(buf, sizeof(buf));
                if ((rval = read(msgsock, buf, 1024)) < 0)
                {
                    perror("reading stream message");
                }
                else if (rval > 0)
                {
                                        
                    stringstream ss;
                    string s;
                    string buff;                    
                    
                    // Buf to stringstream
                    ss << buf;
                    
                    // Create vector to hold our words
                    vector<string> tokens;      
                    
                    // Write to tokens
                    while (ss >> buff)
                        tokens.push_back(buff);
                   
                    // Check the tokens for options
                    if(tokens[0] == "hello"){
                        //
                    }
                    else if(tokens[0] == "mode"){
                        if(tokens[1] == "static")
                            SetMode(1);
                        if(tokens[1] == "dynamic")
                            SetMode(2);
                        if(tokens[1] == "colorfader")
                            SetMode(4);
                            
                        if(m_mode == 1)
                            Log("Mode: Static");
                        else if(m_mode == 2)
                            Log("Mode: Dynamic");
                        else if(m_mode == 3)
                            Log("Mode: RGB Test");
                        else if(m_mode == 4)
                            Log("Mode: Color fader"); 
                    }
                    else if(tokens[0] == "faderbrightness"){
                        SetFaderBrightness(atoi(tokens[1].c_str()));
                    }
                    else if(tokens[0] == "3dmode"){
                        Set3DMode(atoi(tokens[1].c_str()));     
                    }
                    else if(tokens[0] == "cluster"){
                        SetCluster(atoi(tokens[1].c_str()));     
                    }
		            else if(tokens[0] == "delay"){
                        SetDelay(atoi(tokens[1].c_str()));
                        m_delay = atoi(tokens[1].c_str());
                    }
                    else if(tokens[0] == "blackbar"){
                        if(tokens[1] == "true"){
                            SetBlackbar(true);
                        }
                        if(tokens[1] == "false"){
                            SetBlackbar(false);
                            bb_toggle = false;
                            skip_blackYlines = 0;
                            setscanRange(xres,yres);                
                        }
                    }
                    else if(tokens[0] == "interval"){
                        m_interval = atof(tokens[1].c_str());
                        m_timer.SetInterval(Round64(m_interval * 1000000.0));
                    }
                    else if(tokens[0] == "adjust"){
                        int adjust[3];
                        
                        adjust[0] = atoi(tokens[1].c_str());
		                adjust[1] = atoi(tokens[2].c_str());
		                adjust[2] = atoi(tokens[3].c_str());
		                
                        //m_adjust = atof(tokens[1].c_str());
                        if(m_manadjust == true)
                        	boblight_setadjust(m_boblight,adjust);
                    }
                    else if(tokens[0] == "setadjust"){
                        if(tokens[1] == "true"){
                            SetManAdjust(true);
                        }
                        if(tokens[1] == "false"){
                            SetManAdjust(false);             
                        }
                    }
                    else if(tokens[0] == "static_color"){
                        int color;                        
                        //Convert hex to int
                        HexStrToInt(tokens[1].c_str(), color);
                        SetColor(color);
                    }                    
                    else
                    {
                        //Normal options
                        if (!boblight_setoption(m_boblight, -1, buf))
                        {
                            Log("[SocketError] Wrong option or value");
                        }  
                   }
                   
                   if(tokens[0] != "hello" && m_debug)
                       Log("[Message from GUI] Set %s to %s",tokens[0].c_str(),tokens[1].c_str());
                    
                }
          } 
          while (rval > 0);
          close(msgsock);
      }
      close(sock);
      
      // delete socket file
      unlink(NAME);
}

bool CGrabber::detectSTB()
{
	stb_type = UNKNOWN;
	
	// detect STB
	char buf[256];
	FILE *pipe = fopen("/proc/fb","r");
	if (!pipe)
	{
		Log("No framebuffer, unknown STB .. quit.");
		return false;
	}
	
	stb_type = UNKNOWN;

	if (stb_type == UNKNOWN)
	{
		FILE *file = fopen("/proc/stb/info/chipset", "r");
		if (file)
		{
			char buf[32];
			while (fgets(buf, sizeof(buf), file))
			{
				if (strstr(buf,"7400"))
				{
					stb_type = BRCM7400;
					break;
				}
				else if (strstr(buf,"7401"))
				{
					stb_type = BRCM7401;
					break;
				}
				else if (strstr(buf,"7403"))
				{
					stb_type = BRCM7401;
					break;
				}				
				else if (strstr(buf,"7405"))
				{
					stb_type = BRCM7405;
					break;
				}
				else if (strstr(buf,"7413"))
				{
					stb_type = BRCM7405;
					break;
				}
				else if (strstr(buf,"7335"))
				{
					stb_type = BRCM7335;
					break;
				}
				else if (strstr(buf,"7325"))
				{
					stb_type = BRCM7325;
					break;
				}
				else if (strstr(buf,"7358"))
				{
					stb_type = BRCM7358;
					break;
				}
				else if (strstr(buf,"7356"))
				{
					stb_type = BRCM7356;
					break;
				}
				else if (strstr(buf,"7424"))
				{
					stb_type = BRCM7424;
					break;
				}
				else if (strstr(buf,"7425"))
				{
					stb_type = BRCM7425;
					break;
				}
			}
			fclose(file);
		}
	}

	if (stb_type == UNKNOWN)
	{
		FILE *file = fopen("/proc/stb/info/model", "r");
		if (file)
		{
			char buf[32];
			while (fgets(buf, sizeof(buf), file))
			{
				if (strcasestr(buf,"DM500HD") || strcasestr(buf,"DM800SE") || strcasestr(buf,"DM7020HD"))
				{
					stb_type = BRCM7405;
					break;
				}
				else if (strcasestr(buf,"DM8000"))
				{
					stb_type = BRCM7400;
					break;
				}
				else if (strcasestr(buf,"DM800"))
				{
					stb_type = BRCM7401;
					break;
				}
				else if (strcasestr(buf,"Gigablue"))
                {
                   stb_type = BRCM7335;
                   break;
                }
			}
			fclose(file);
		}
	}
	

	if (stb_type == UNKNOWN) {
		return false;
	}

	switch (stb_type)
	{
		case BRCM7400:
			registeroffset = 0x10100000;
			chr_luma_stride = 0x40;
			chr_luma_register_offset = 0x20;
			mem2memdma_register = 0x10c02000;
			break;
		case BRCM7401:
			registeroffset = 0x10100000;
			chr_luma_stride = 0x40;
			chr_luma_register_offset = 0x20;
			mem2memdma_register = 0;
			break;
		case BRCM7405:
			registeroffset = 0x10100000;
			chr_luma_stride = 0x80;
			chr_luma_register_offset = 0x20;
			mem2memdma_register = 0;
			break;
		case BRCM7325:
			registeroffset = 0x10100000;
			chr_luma_stride = 0x80;
			chr_luma_register_offset = 0x20;
			mem2memdma_register = 0;
			break;
		case BRCM7335:
			registeroffset = 0x10100000;
			chr_luma_stride = 0x40;
			chr_luma_register_offset = 0x20;
			mem2memdma_register = 0x10c01000;
			break;
		case BRCM7358:
			registeroffset = 0x10600000;
			chr_luma_stride = 0x40;
			chr_luma_register_offset = 0x34;
			mem2memdma_register = 0;
			break;
		case BRCM7356:
			registeroffset = 0x10600000;
			chr_luma_stride = 0x80;
			chr_luma_register_offset = 0x34;
			mem2memdma_register = 0;
			break;
		case BRCM7424:
		case BRCM7425:
			registeroffset = 0x10600000;
			chr_luma_stride = 0x80;
			chr_luma_register_offset = 0x34;
			mem2memdma_register = 0;
			break;
		default:
			break;
	}

	
	return true;
}

/*
Basic concept of the blackbar detection:
 
 - look at two position on upper side of video screen, if there are black
 colored pixels. If so, move line downward to see where blackbar ends, then
 write number of lines into variable "sb". sb is then the offset for
 colorcalulcation in the main program.
*/

void CGrabber::beamProcess()
{
   	int x,y,xpos,Yblack_lines;
    
    int xres_ = xres;
    
    if(m_blackbar)
    {

            y=0;Yblack_lines=0;bb_black=0;bb_color=0; //set all vars to 0        
            while (bb_color < 1)
            {
	            Yblack_lines++; // Move line downward
	            
	            //Look at left to middle
	            for (xpos=0; xpos < xres_ /2; xpos++)
	            {
	                if ((luma[xpos + Yblack_lines * xres_ ] > 22) || (chroma[(xpos&~1) + (Yblack_lines>>1)*xres_ ] > 133) || (chroma[(xpos&~1) + (Yblack_lines>>1)*xres_  + 1] > 133) || (Yblack_lines > (yres/2)))
	                    {bb_color =1;}else{bb_black++;}                        
	            }
	            
            }
            
            //if(m_debug)
                //printf("Look at left to middle over xres: %d -> Yblack_lines: %d Blacklines:%d\n",xres_,Yblack_lines,bb_black);
            
            
            //
            // Calculate
            //
            if ((abs(bb_lines - bb_black) < 4) && (bb_black >= 2)) {
                bbv_++;
                if (bbv_ >10) 
                    bbv_=10;
            } 
            else 
            {
                bbv_--;
                if(bbv_ <0) 
                    bbv_=0;
            };
            
		    bb_lines = bb_black;


            //
            y=0;Yblack_lines=0;bb_black=0;bb_color=0; //set all vars to 0                
            while (bb_color < 1)
            {
	            Yblack_lines++; // Move line downward
                
                //Look from middle to right            
	            for (xpos=xres_ /2; xpos < xres_ ; xpos++)
	            {
	                if ((luma[xpos + Yblack_lines * xres_ ] > 22) || (chroma[(xpos&~1) + (Yblack_lines>>1)*xres_ ] > 133) || (chroma[(xpos&~1) + (Yblack_lines>>1)*xres_  + 1] > 133) || (Yblack_lines > (yres/2)))
	                    {bb_color =1;}else{bb_black++;} 
	            }                   
	            
            }
            
            //if(m_debug)
                //printf("Look from middle to right over xres: %d -> Yblack_lines: %d Blacklines:%d\n",xres_,Yblack_lines,bb_black);
            
            //
            // Calculate
            //
            
            if ((abs(bb_lines_ - bb_black) < 4) && (bb_black >= 2)) {
                bbv__++;
                if (bbv__ >10) 
                    bbv__=10;
            } 
            else 
            {
                bbv__--;
                if(bbv__ <0) 
                    bbv__=0;
            };
            
		    bb_lines_ = bb_black;
		

            if ((bbv_ >= 10 && bbv__ >=10 && Yblack_lines > 2) && !bb_toggle) {
                
                skip_blackYlines = Yblack_lines;                
                blackYLines = ((bb_lines + bb_lines_)>>1)+1;
                
                bb_toggle=true;
                bb_old = blackYLines;
                Log ("[Message] Black bars detected. YLines:%d",skip_blackYlines);
                setscanRange(xres_ ,yres-(skip_blackYlines*2));
            }
            

            if ((bbv_ < 2 && bbv__ < 2 && Yblack_lines <= 2) && bb_toggle) {
                skip_blackYlines=blackYLines = bb_lines=bb_lines_=bbv_=bbv__=0;
                
                bb_toggle=false;
                Log ("[Message] Black bars no longer detected. YLines:%d",skip_blackYlines);
                setscanRange(xres_ ,yres);
            }
        
    }
}    

