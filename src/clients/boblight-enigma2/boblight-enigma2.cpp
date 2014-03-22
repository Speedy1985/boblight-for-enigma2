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

#define BOBLIGHT_DLOPEN_EXTERN
#include "lib/boblight.h"

#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <sstream>
#include <stdlib.h>

#include "util/misc.h"
#include "util/timer.h"
#include "util/daemonize.h"

#include "config.h"
#include "grabber-enigma2.h"

int  Run();
void SignalHandler(int signum);

volatile bool stop = false;

using namespace std;

CFlagManagerEnigma2 g_flagmanager;

int main (int argc, char *argv[])
{

  //
  // Check if client already running...
  //
  
  pid_t pid = getpid();

  std::stringstream command;
  command << "ps aux | grep boblight-enigma2 | grep -v 'grep' | grep -v 'gdb' | grep -v " << pid;
  int isRuning = system(command.str().c_str());
  if (isRuning == 0) {
      cout << "Boblight-enigma2 is already running, Exit..." << endl;
      exit(1);
  }
    
  //load the boblight lib, if it fails we get a char* from dlerror()
  char* boblight_error = boblight_loadlibrary(NULL);
  if (boblight_error)
  {
    PrintError(boblight_error);
    return 1;
  }

  //try to parse the flags and bitch to stderr if there's an error
  try
  {
    g_flagmanager.ParseFlags(argc, argv);
  }
  catch (string error)
  {
    PrintError(error);
    //g_flagmanager.PrintHelpMessage();
    return 1;
  }
  
  if (g_flagmanager.m_printhelp) //print help message
  {
    g_flagmanager.PrintHelpMessage();
    return 1;
  }

  if (g_flagmanager.m_printboblightoptions) //print boblight options (-o [light:]option=value)
  {
    g_flagmanager.PrintBoblightOptions();
    return 1;
  }
  
  //set up signal handlers
  signal(SIGTERM, SignalHandler);
  signal(SIGINT, SignalHandler); 
  
  printf("\nBoblight Grabber - Boblight for enigma2 %s (c) 2012-2014 Speedy1985 and Oktay Oeztueter)\n",PACKAGE_VERSION);
  
  if (g_flagmanager.m_fork)
    Daemonize();           

  //keeps running until some unrecoverable error happens
  return Run();
}

int Run()
{
    
  while(!stop)
  {
    //init boblight
    void* boblight = boblight_init();
    
    cout << "Connecting to boblightd\n";
    
    //try to connect, if we can't then bitch to stderr and destroy boblight
    if (!boblight_connect(boblight, g_flagmanager.m_address, g_flagmanager.m_port, 5000000) ||
        !boblight_setpriority(boblight, g_flagmanager.m_priority))
    {
      PrintError(boblight_geterror(boblight));
      cout << "Waiting 10 seconds before trying again\n";
      boblight_destroy(boblight);
      sleep(10);
      continue;
    }
    
    cout << "Connection to boblightd opened\n";
        
    //Set gamma, if gamma is not set then use standard 2.2 since this is standard for video.
    g_flagmanager.SetVideoGamma();
  
    //if we can't parse the boblight option lines (given with -o) properly, just exit
    try
    {
      g_flagmanager.ParseBoblightOptions(boblight);
    }
    catch (string error)
    {
      PrintError(error);
      return 1;
    }

	CGrabber* grabber;
    
    grabber = new CGrabber(boblight, stop, g_flagmanager.m_sync);

    grabber->SetInterval(g_flagmanager.m_interval);             // Set interval.
    grabber->SetSize(g_flagmanager.m_pixels);                   // Set maximum size for the intermediate image.
    grabber->SetDebug(g_flagmanager.m_debug);                   // Set debug enabled or disabled.
    grabber->SetPicdump(g_flagmanager.m_picdump);               
    grabber->Set3DMode(g_flagmanager.m_3d_mode); 
    grabber->SetFaderBrightness(g_flagmanager.m_brightness); 
    grabber->SetGrabInfo(g_flagmanager.m_grabinfo);             // Testing....
    grabber->SetBlackbar(g_flagmanager.m_blackbar);		        // Set blackbardetection enabled or disabled/
    grabber->SetDelay(g_flagmanager.m_delay);			        // Set delay for slow hdmi transfer
    grabber->SetCluster(g_flagmanager.m_cluster);               // Set cluster
    grabber->SetMode(g_flagmanager.m_mode);                     // Mode
    grabber->SetColor(g_flagmanager.m_color);                   // Color for static
	grabber->SetManAdjust(g_flagmanager.m_use_manual_adjust);   // Manual Ajust on or off

    boblight_fillbuffer(boblight);                              // Fill colorbuffer for delayfunction
    
    //if(g_flagmanager.m_adjust[0] + g_flagmanager.m_adjust[1] + g_flagmanager.m_adjust[2] > 0)
        //cout << "Set Custom rgb adjustment:     " << g_flagmanager.m_adjust[0] << " " << g_flagmanager.m_adjust[1] <<  " " << g_flagmanager.m_adjust[2] << "\n";
    
    //Use UdjustFromGui, only support on Enigma2 with Boblightd from Speedy1985
    if(g_flagmanager.m_use_manual_adjust == true)
    	boblight_setadjust(boblight,g_flagmanager.m_adjust);
                             
    if (!grabber->Setup()) //just exit if we can't set up the grabber
    {
      PrintError(grabber->GetError());
      delete grabber;
      boblight_destroy(boblight);
      return 1;
    }

    if (!grabber->Run()) //just exit if some unrecoverable error happens
    {
      //PrintError(grabber->GetError());
      delete grabber;
      boblight_destroy(boblight);
      return 1;
    }
    else //boblightd probably timed out, so just try to reconnect
    {
      if (!grabber->GetError().empty())
        PrintError(grabber->GetError());
    }

    delete grabber;

    boblight_destroy(boblight);
	
    cout << "Exit..\n";
  }

  return 0;
}

void SignalHandler(int signum)
{
  if (signum == SIGTERM)
  {
    cout << "caught SIGTERM\n";
    stop = true;
  }
  else if (signum == SIGINT)
  {
    cout << "caught SIGINT\n";
    stop = true;
  }
}
