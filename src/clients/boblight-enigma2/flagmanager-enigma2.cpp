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

#include <iostream>

#include "flagmanager-enigma2.h"
#include "util/misc.h"
#include "config.h"
#include <stdlib.h>

using namespace std;

CFlagManagerEnigma2::CFlagManagerEnigma2()
{
  //extend the base getopt flags
  //i = interval, u = pixels, d = debug
  m_flags += "a:x:i:t:w:r:m:j:c:d::b::k::g::u::";

  m_cluster     = 1;         // Default set 1 for cluster
  m_blackbar    = false;	 // Blackbar detection, default disabled
  m_interval    = 0.1;   	 // default interval is 100 milliseconds
  //m_pixels 	    = 16;   	 // Set pixel default.
  m_delay       = 0;
  m_debug 	    = false; 	 // no debugging by default
  m_grabinfo    = false;     // for debug
  m_picdump     = false;  	 // Picture dump, default false
  m_sync 	    = true;      // sync mode, enabled by default
  m_mode        = 2;         // dynamic or static, default is dynamic  
  m_brightness  = 255;       // Fader brightness
  m_3d_mode     = 1;         // normal
  
  for(int i=0; i < 3; i++){
    m_adjust[i] = 0;
  } 
  
  m_use_manual_adjust = true;
}

std::vector<std::string> split(std::string str,std::string sep){
    char* cstr=const_cast<char*>(str.c_str());
    char* current;
    std::vector<std::string> arr;
    current=strtok(cstr,sep.c_str());
    while(current!=NULL){
        arr.push_back(current);
        current=strtok(NULL,sep.c_str());
    }
    return arr;
}

void CFlagManagerEnigma2::ParseFlagsExtended(int& argc, char**& argv, int& c, char*& optarg)
{
  if (c == 'a') //adjust
  {
    
    std::vector<std::string> arr;
    arr=split(string(optarg),"/");
    if(arr.size() < 3 || arr.size() > 3)
    {
        throw string("Wrong value " + string(optarg) + " for adjust");
    }
    else{
        for(size_t i=0;i<arr.size();i++){
            m_adjust[i] = Clamp(atoi(arr[i].c_str()),0,255);
        }
    }
    //
    
  }
  else if (c == 'x') //cluster
  {
    if (!StrToInt(optarg, m_cluster) || m_cluster <= 0 || m_cluster > 10)
    {
      throw string("Wrong value " + string(optarg) + " for cluster");
    }
  }
  else if (c == 'i') //interval
  {
    if (!StrToFloat(optarg, m_interval) || m_interval <= 0.0)
    {
      throw string("Wrong value " + string(optarg) + " for interval");
    }
  }
  else if (c == 't') //delay to use
  {
    if (!StrToInt(optarg, m_delay) || m_delay <= 0)
    {
      throw string("Wrong value " + string(optarg) + " for delay");
    }
  }
  else if (c == 'r') //beta 3dmode
  {
    if (!StrToInt(optarg, m_3d_mode) || m_3d_mode <= 0)
    {
      throw string("Wrong value " + string(optarg) + " for 3dmode");
    }
  }
  else if (c == 'm') // mode
  {
    if (!StrToInt(optarg, m_mode) || m_mode <= 0)
    {
      throw string("Wrong value " + string(optarg) + " for mode");
    }
  }
  else if (c == 'j') // fader brightness
  {
    if (!StrToInt(optarg, m_brightness) || m_brightness <= 0)
    {
      throw string("Wrong value " + string(optarg) + " for brightness");
    }
  }
  else if (c == 'c') // mode
  {  
    if (!HexStrToInt(optarg, m_color) || m_color & 0xFF000000)
    {
        throw string("wrong value " + string(optarg) + " for color");
    }
  }
  else if (c == 'd') //turn on debug mode
  {
    m_debug = true;
  }
  else if (c == 'b') //turn on blackbar mode
  {
    m_blackbar = true;
  }
  else if (c == 'k') //turn on grabinfo mode
  {
    m_grabinfo = true;
  }
  else if (c == 'g') //turn on picdump mode
  {
    m_picdump = true;
  }
  else if (c == 'u') //turn on manual adjust mode (only for enigma2 with boblightd from speedy1985)
  {
    m_use_manual_adjust = false;
  }
}

void CFlagManagerEnigma2::PrintHelpMessage()
{
  cout << "Usage: boblight-enigma2 [OPTION]\n";
  cout << "\n";
  cout << "  options:\n";
  cout << "\n";
  cout << "  -p  priority, from 0 to 255, default is 128\n";
  cout << "  -s  address:[port], set the address and optional port to connect to\n";
  cout << "  -o  add libboblight option, syntax: [light:]option=value\n";
  cout << "  -l  list libboblight options\n";
  cout << "\n";
  cout << "  -i  set the interval in mseconds, default is 0.1 (100 milliseconds)\n";
  
  cout << "  -a  adjust RGB, Example: -a 255/255/30 [default it wil use boblight.conf settings]\n";
  cout << "  -u  disable adjust option for other deamons [Default it is enable]\n";
  cout << "  -m  mode, 1 = static, 2 = dynamic, 3 = test rgb, 4 = rgb fader, [default 2]\n";
  cout << "  -x  cluster leds, cluster the X leds as one, [default 1]\n";
  cout << "  -c  color for static mode, is in RRGGBB hex notation [default 000000]\n";
  cout << "  -j  fader brightness [default 255]\n";
  
  cout << "  -t  delay, set a syncdelay. for some tv's with slow hdmi buffering.\n";
  cout << "  -b  blackbar mode, default disabled\n";
  cout << "  -g  picture dump, this option saves a lot of pictures from grabber to /tmp. \n";
  cout << "  -r  3d mode, 1 = normal, 2 = top and bottom, 3 = sidebyside, [default 1] \n";
  cout << "  -d  enable debug mode. \n";
  cout << "  -k  enable grabinfo (offset,address). \n";
  cout << "  -f  fork\n";
  cout << "\n";
}
