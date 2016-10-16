/*
  servo.h - servo motor driver: executes motion plans of planner.c using the servo motors
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

#ifndef servo_h
#define servo_h 

// Nice functions to have predefined
#define BIT(x) (1 << (x))
#define SETBITS(x,y) ((x) |= (y))
#define CLEARBITS(x,y) ((x) &= (~(y)))
#define SETBIT(x,y) SETBITS((x), (BIT((y))))
#define CLEARBIT(x,y) CLEARBITS((x), (BIT((y))))
#define BITSET(x,y) ((x) & (BIT(y)))
#define BITCLEAR(x,y) !BITSET((x), (y))
#define BITSSET(x,y) (((x) & (y)) == (y))
#define BITSCLEAR(x,y) (((x) & (y)) == 0)
#define BITVAL(x,y) (((x)>>(y)) & 1)

#define MIN(a,b)              ((a<b)?(a):(b))
#define MAX(a,b)              ((a>b)?(a):(b))
#define ABS(x)                ((x>0)?(x):(-x))
#define LIMIT(x,low,high)     ((x>high)?(high):((x<low)?(low):(x)))

#ifndef SEGMENT_BUFFER_SIZE
  #define SEGMENT_BUFFER_SIZE 6
#endif

/*
 * PWM definitions
 */

#define FULL_PWM 		ICR1
#define M1_PWM	 		OCR1B
#define M2_PWM 			OCR1A

/*
 * Motor control subroutines
 */

// On disable, we disable the pin change interrupt, set the DIAG pin as output and pull the DIAG line low
//#define M1_DISABLE			CLEARBIT(PCMSK0, M1_DIAG_A); CLEARBIT(PCMSK0, M1_DIAG_B); SETBIT(M1_DIAG_A_DDR, M1_DIAG_A); SETBIT(M1_DIAG_B_DDR, M1_DIAG_B); CLEARBIT(M1_DIAG_A_PORT, M1_DIAG_A); CLEARBIT(M1_DIAG_B_PORT, M1_DIAG_B)
//#define M1_ENABLE			CLEARBIT(M1_DIAG_A_DDR, M1_DIAG_A); CLEARBIT(M1_DIAG_B_DDR, M1_DIAG_B); SETBIT(PCMSK0, M1_DIAG_A); SETBIT(PCMSK0, M1_DIAG_B)

// Note that CW and CCW are basically reversed for backwards compatability (changes Polarity setting)
#define M1_CCW				SETBIT(M1_IN_A_PORT, M1_IN_A); CLEARBIT(M1_IN_B_PORT, M1_IN_B)
#define M1_CW				CLEARBIT(M1_IN_A_PORT, M1_IN_A); SETBIT(M1_IN_B_PORT, M1_IN_B)
#define M1_STOP_GND			CLEARBIT(M1_IN_A_PORT, M1_IN_A); CLEARBIT(M1_IN_B_PORT, M1_IN_B)
#define M1_STOP_VCC			SETBIT(M1_IN_A_PORT, M1_IN_A); SETBIT(M1_IN_B_PORT, M1_IN_B)

/*
 * Enumeration definitions
 */

#define ENABLE_OFF			0
#define ENABLE_ON			1

#define MIXED_MODE_STATE_OFF	0
#define MIXED_MODE_STATE_POS	1
#define MIXED_MODE_STATE_VEL	2

#define STREAM_MODE_OFF		0
#define STREAM_MODE_ON		1

#define CONTROL_MODE_POS	0
#define CONTROL_MODE_VEL	1
#define CONTROL_MODE_MIXED	2

#define FEEDBACK_MODE_ENC	0
#define FEEDBACK_MODE_POT	1

#define POLARITY_REGULAR	0
#define POLARITY_FLIPPED	1

#define OUTPUT_DIRECTION_CW		0
#define OUTPUT_DIRECTION_CCW	1
#define OUTPUT_DIRECTION_NONE	2


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
