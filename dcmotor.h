/*
 * This file is part of the MCBMini firmware.
 * MCBMini is a complete, open-source, flexible and scalable 
 * motor control scheme with board designs, firmware and host 
 * software. 
 * This is the firmware for MCBMini
 * The MCBMini project can be downloaded from:
 * http://code.google.com/p/mcbmini/ 
 *
 * (c) Sigurdur Orn Adalgeirsson (siggi@alum.mit.edu)
 *
 * MCBMini firmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License
 * 
 * MCBMini firmware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with the MCBMini firmware.  If not, see <http://www.gnu.org/licenses/>.
 */
 
 /*
 * motor.h
 *
 *  Created on: Jul 25, 2012
 *      Author: siggi
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "mcb_mini.h"

/*
 * Enumeration definitions
 */

#define DIRECTION_CW		0
#define DIRECTION_CCW		1
#define DIRECTION_STOP		2

/*
 * The actual motor structure
 */

typedef struct{
	uint8_t motor_index;

	int32_t actual_pot;
	int32_t motor_current;
	int32_t actual_enc;

	int32_t old_enc;
	uint8_t last_direction;

	/*
	 * Variables that are used to keep intermediate track of feedback
	 */
	uint16_t current_pot_acc;
	uint16_t motor_current_acc;
	int16_t encoder_increment;

	/*
	 * Vars that control the slow enable process of the motor
	 */
	uint8_t slow_enable_step;
	uint8_t slow_enable_step_counter;
	uint8_t slow_enable_step_time;

	/*
	 * Vars that control the extra pin functionality
	 */
	uint8_t extra_switch;
	uint8_t extra_mode;
	uint8_t extra_servo;
	uint16_t extra_analog;
	uint16_t extra_analog_acc;

} Motor;

void changeDirection(volatile Motor* motor, uint8_t new_direction){
	if( new_direction == motor->last_direction ){
		return;
	}

	// Set direction positive
	if( new_direction == DIRECTION_CW ){
		if(motor->motor_index==0)	{ M1_CW; }
		else						{ M2_CW; }
	}
	// Set direction negative
	else if( new_direction == DIRECTION_CCW ){
		if(motor->motor_index==0)	{ M1_CCW; }
		else						{ M2_CCW; }
	}
	// Set direction stop
	else if( new_direction == DIRECTION_STOP ){
		if(motor->motor_index==0)	{ M1_STOP_GND; }
		else						{ M2_STOP_GND; }
	}

	motor->last_direction = new_direction;
}

void initMotor(volatile Motor *motor, uint8_t motor_index_){
	motor->motor_index = motor_index_;
	motor->last_direction = 99;				 // A value that is invalid on purpose
	changeDirection(motor, DIRECTION_STOP);
	motor->slow_enable_step_time = 30;
	motor->slow_enable_step_counter = motor->slow_enable_step_time;
	motor->extra_mode = EXTRA_MODE_OFF;
}

#endif /* MOTOR_H_ */
