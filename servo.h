/*
  servo.h - servo motor driver: executes motion plans of planner.c using the servo motors
  Part of Horus Firmware

  Copyright (c) 2014-2015 Mundo Reader S.L.
  Copyright (c) 2016 Jeremy Reeve

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

#ifndef servo_h
#define servo_h 

#ifndef SEGMENT_BUFFER_SIZE
  #define SEGMENT_BUFFER_SIZE 6
#endif

// Initialize and setup the servo motor subsystem
void servo_init();

// Enable servos, but cycle does not start unless called by motion control or runtime command.
void servo_wake_up();

// Sets a flag to disable/enable motors on st_go_idle() function.
void servo_disable_on_idle(uint8_t);

// Immediately disables servos
void servo_go_idle();

// Generate the step and direction port invert masks.
void servo_generate_step_dir_invert_masks();

// Reset the servo subsystem variables       
void servo_reset();
             
// Reloads step segment buffer. Called continuously by runtime execution system.
void servo_prep_buffer();

// Called by planner_recalculate() when the executing block is updated by the new plan.
void servo_update_plan_block_parameters();

// Called by runtime status reporting if realtime rate reporting is enabled in config.h.
#ifdef REPORT_REALTIME_RATE
float servo_get_realtime_rate();
#endif

#endif
