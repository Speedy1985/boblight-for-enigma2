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

#ifndef FLAGMANAGERENIGMA2
#define FLAGMANAGERENIGMA2


#include "clients/flagmanager.h"
	
class CFlagManagerEnigma2 : public CFlagManager
{
  public:
    CFlagManagerEnigma2();
    
    void        ParseFlagsExtended(int& argc, char**& argv, int& c, char*& optarg); //we load our own flags here
    void        PrintHelpMessage();

    double      m_interval;           //grab interval in seconds, or vertical blanks when negative
    int         m_pixels;             //number of pixels on lines to capture
    bool        m_debug;              
    bool        m_picdump;
    bool        m_blackbar;
    bool        m_grabinfo;
    int         m_mode;
    int         m_brightness;
    int         m_cluster;
    int         m_color;
    int         m_3d_mode;
    int         m_delay;
    int         m_adjust[3];
    bool		m_use_manual_adjust;
};

#endif //FLAGMANAGERENIGMA2
