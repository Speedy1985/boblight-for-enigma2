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

#ifndef CGRABBER
#define CGRABBER

#include <string>
#include <sys/resource.h>

#include "config.h"
#include "util/timer.h"
#include "util/mutex.h"
#include "util/threading.h"
#include "flagmanager-enigma2.h"

#define CLAMP(x)    ((x < 0) ? 0 : ((x > 255) ? 255 : x))
#define SWAP(x,y)	{ x ^= y; y ^= x; x ^= y; }
#define CLIP(x)     ((x < 0) ? 1 : ((x > 250) ? 250 : x))

#define RED565(x)    ((((x) >> (11 )) & 0x1f) << 3)
#define GREEN565(x)  ((((x) >> (5 )) & 0x3f) << 2)
#define BLUE565(x)   ((((x) >> (0)) & 0x1f) << 3)

#define YFB(x)    ((((x) >> (10)) & 0x3f) << 2)
#define CBFB(x)  ((((x) >> (6)) & 0xf) << 4)
#define CRFB(x)   ((((x) >> (2)) & 0xf) << 4)
#define BFFB(x)   ((((x) >> (0)) & 0x3) << 6)

#define MIN(a, b) ((a) < (b)) ? (a) : (b)

// dont change SPARE_RAM and DMA_BLOCKSIZE until you really know what you are doing !!!
#define SPARE_RAM 252*1024*1024 // 0XFC00000 // the last 4 MB is enough...
#define DMA_BLOCKSIZE 0x3FF000 // should be big enough to hold a complete YUV 1920x1080 HD picture, otherwise it will not work properly on DM8000

// STB-Types
#define UNKNOWN     0
#define PALLAS      0
#define VULCAN      0
#define XILLEON     0
#define BRCM7325    7325
#define BRCM7335    7335
#define BRCM7346    7346
#define BRCM7356    7356
#define BRCM7358    7358
#define BRCM7400    7400
#define BRCM7401    7401
#define BRCM7403    7403
#define BRCM7405    7405
#define BRCM7424    7424
#define BRCM7425    7425
#define GIGABLUE    0//12

// for writing BMP files
#define PUT32(x) hdr[i++] = ((x)&0xFF); hdr[i++] = (((x)>>8)&0xFF); hdr[i++] = (((x)>>16)&0xFF); hdr[i++] = (((x)>>24)&0xFF);
#define PUT16(x) hdr[i++] = ((x)&0xFF); hdr[i++] = (((x)>>8)&0xFF);
#define PUT8(x) hdr[i++] = ((x)&0xFF);

// Socket for communication with GUI
#define NAME "/tmp/boblight-gui.socket"

class CGrabber :  public CThread
{

  	public: 
        CGrabber(void* boblight, volatile bool& stop, bool sync);
        ~CGrabber();

        std::string& GetError()           { return m_error; }        	  // retrieves the latest error  
        
        void SetInterval(double interval) { m_interval  = interval; } 	  // sets interval, negative means vblanks
        void SetSize(int size)            { m_size      = size; }         // sets how many pixels we want to grab maximum
        void SetDelay(int delay)          { m_delay     = delay; }        
        void SetDebug(bool debug)         { m_debug     = debug; }        // sets debug mode
        void SetPicdump(bool picdump)     { m_picdump   = picdump; }   	  // sets picdump mode
        void SetBlackbar(bool blackbar)   { m_blackbar  = blackbar; } 	  // sets blackbar mode to enabled
        void Set3DMode(int _3dmode)       { m_3d_mode  = _3dmode; }       // 
        void SetGrabInfo(bool grabinfo)   { m_grabinfo  = grabinfo; }     // 
        void SetMode(int mode)            { m_mode      = mode; }         // Mode, dynamic or static 
        void SetFaderBrightness(int brightness) { m_brightness = brightness; }  // Mode, fader brighness
        void SetColor(int color)          { m_color     = color; }        // Color for static
        void SetCluster(int cluster)      { m_cluster   = cluster; }      // Set cluster
        void SetManAdjust(bool manadjust) { m_manadjust = manadjust; }	
        bool Setup();                                                	  // base setup function
        bool Run();                       				             	  // main run function
            
    protected:
        
        
        //
        //Cluster leds
        //
        int             m_cluster;
        
        //
        // Static mode
        //
        int             m_color;
        
        //
        // Threads
        //
        void            socketProcess();    // Socket connection    
        
        //
        // convert the video from YUV to RGB
        //
        bool            CovertVideo(int yStart);
        
        //
        // the actual grabber
        //
        int             buffer_size;
        int             chr_luma_stride;
        int             chr_luma_register_offset;
        unsigned int    registeroffset;
        unsigned int    mem2memdma_register;
        bool            grabVideo();
        long double     m_last_res_process;
        
        //
        // helper functions. taken from aiograb
        //
        int             hexFromFile(const char *filename);
        const char      *file_getline(const char *filename);
        int             file_scanf_line(const char *filename, const char *fmt, ...);
        int             file_scanf_lines(const char *filename, const char *fmt, ...);
        
        
        // Check scanrange for 3dmode or normal
        void            setscanRange(int xres, int yres);
        int             checkResolution(int stride, int *xres, int *yres, long double now);
        
        //
        // detect the Type of the Box
        //
        bool            detectSTB();
        int             stb_type;

        int             mem_fd;			                        // handle to the memory
        bool            blank;			                        // blank signal bool 
        int             xres_orig, yres_orig;	                // original resolution
        int             xres, yres, xres_tmp, yres_tmp;			// final resolution
        int             adjust_x, adjust_y;                     // 
        int             xres_old, yres_old;	                    // stored to detect resolution changes
        int             skiplines;		                        // downscale-factor (power of two)
        unsigned char   *luma, *chroma, *video;                 // buffer for chroma and luma data
        bool            set_black;
        bool            give_error;
        
        
        //fps
        long double     m_lastupdate;
        long double     m_lastmeasurement;
        long double     m_measurements;
        int             m_nrmeasurements;
        long double     m_fps;        
        int64_t			fps_lastupdate;
        int				fps_framecount;
        
        // save grabbed image as PNG
        bool       		SaveBMP(int xres, int yres, int output_bytes);
        int        	 	filename_counter;
        int        	 	filename_count;
        
        // Timer
        double          m_interval;                                // interval in seconds, or negative for vblanks
        CTimer          m_timer;                                   // our timer

        std::string     m_error;                                   // latest error
        int             m_mode;                                    // static or dynamic
        int             m_brightness;
        int             m_3d_mode;
        int             m_old_3d_mode;
        
        // ---
        int             m_size;                                    // nr of pixels on lines to grab
        bool            m_debug;                                   // if we have debug mode on
        bool            m_picdump;                                 // if we have picdump mode on
        bool            m_blackbar;
        int             m_delay;
        int             m_adjust;
        bool			m_manadjust;
        void            UpdateDebugFps();
        void            writeFPS();                                   
        bool			m_sync;                                    // sync mode for libboblight
        int             nrlights;                                  // Lights
        volatile bool&  m_stop;
        void*           m_boblight;                                // our handle from libboblight
        bool            m_lightsoff;
        
        
        // Options from arg
        bool            m_grabinfo;
        
        //
        // Blackbar detection
        //
        long double     m_last_beam_process;
        void            beamProcess();
        int             blackYLines,bb_black,bbv_,bbv__,bb_color,beamcount,bb_lines,bb_lines_,bb_old,skip_blackYlines,bb;
        bool            bb_toggle;
    
};
#endif //CGRABBER
