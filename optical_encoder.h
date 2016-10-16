/*
  optical_encoder.h - servo motor driver: executes motion plans of planner.c using the servo motors
  Part of Horus Firmware

  Copyright (c) 2014-2015 Mundo Reader S.L.
  Copyright (c) 2016 Jeremy Simas Reeve

  Horus Firmware is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Horus Firmware is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Horus Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/
/* 
  This file is based on work from Grbl v0.9, distributed under the 
  terms of the GPLv3. See COPYING for more details.
    Copyright (c) 2009-2011 Simen Svale Skogsrud
    Copyright (c) 2011-2014 Sungeun K. Jeon
*/

#ifndef optical_encoder_h
#define optical_encoder_h 

#ifndef SEGMENT_BUFFER_SIZE
  #define SEGMENT_BUFFER_SIZE 6
#endif

// Initialize and setup the optical encoder subsystem
void oe_init();

// Reset the optical encoder subsystem variables       
void oe_reset();
             
// Called by runtime status reporting if realtime rate reporting is enabled in config.h.
#ifdef REPORT_REALTIME_RATE
float oe_get_realtime_rate();
#endif

#endif
