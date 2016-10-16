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
 * pid.h
 *
 *  Created on: Jul 25, 2012
 *      Author: siggi
 */

#ifndef PID_H_
#define PID_H_

#include <util/atomic.h>
#include <limits.h>


#define I_COMP_MAX 		10000L << 4
#define D_ERROR_MAX 	1000L

#define SATURATION_NONE		0
#define SATURATION_BOTTOM	1
#define SATURATION_TOP		2

/*
 * The actual PID structure
 */
typedef struct{
	uint16_t p_gain;
	uint16_t d_gain;
	uint16_t i_gain;

	uint8_t downscale;
	uint16_t max_output;

	uint8_t saturation;
	int32_t i_component;

	int32_t old_error;
} Pid;

/*
 * Should be called whenever a change happens in the input signal (change in source or something)
 */
void pidClearState(volatile Pid *pid){
	pid->i_component = 0;
	pid->old_error = LONG_MAX;
	pid->saturation = SATURATION_NONE;
}

int32_t pidCalculateOutput(volatile Pid *pid, int32_t target, int32_t actual){
	int32_t d_error, error, temp32, output;

	/*
	 * Calculate the error signal and the difference in error signal
	 */
	error = target - actual;

	if( pid->old_error == LONG_MAX ){
		pid->old_error = error;
	}

	temp32 = D_ERROR_MAX << pid->downscale;
	d_error = LIMIT( error - pid->old_error, -temp32, temp32);

	pid->old_error = error;

	temp32 = 0;
	// Here we do the saturation trick  described here: http://www.embeddedrelated.com/showarticle/123.php
	if( (error < 0 && pid->saturation == SATURATION_BOTTOM) || (error > 0 && pid->saturation == SATURATION_TOP) ){
		temp32 = 0;	// Do nothing
	}
	// Here we do an experimental thing which is to reduce the integrative component faster when it is working against our error (not to catch up anymore like in a steady state correction)
	else if( (error < 0 && pid->i_component > 0) || (error > 0 && pid->i_component < 0) ){
		ATOMIC_BLOCK(ATOMIC_FORCEON){
			temp32 = LIMIT( (int32_t)pid->i_gain*error, LONG_MIN, LONG_MAX );
		}
		temp32 = LIMIT( temp32 << 3, LONG_MIN, LONG_MAX ); // Multiply by eight
	}
	else{
		ATOMIC_BLOCK(ATOMIC_FORCEON){
			temp32 = LIMIT( pid->i_gain*error, LONG_MIN, LONG_MAX );
		}
	}
	pid->i_component = LIMIT( pid->i_component + temp32 , LONG_MIN, LONG_MAX );

	// Saturation on the i_component
	temp32 = I_COMP_MAX << pid->downscale;
	pid->i_component = LIMIT( pid->i_component, -temp32, temp32 );


	/*
	 * Assemble PID output
	 */
	// For some reason everything needs to be in its own if statement
	ATOMIC_BLOCK(ATOMIC_FORCEON){
		output = LIMIT( (int32_t)pid->p_gain*error, LONG_MIN, LONG_MAX );
	}

	ATOMIC_BLOCK(ATOMIC_FORCEON){
		temp32 = LIMIT( (int32_t)pid->d_gain*d_error, LONG_MIN, LONG_MAX );
	}
	output = LIMIT( output + temp32, LONG_MIN, LONG_MAX );

	ATOMIC_BLOCK(ATOMIC_FORCEON){
		output = LIMIT( output + (pid->i_component >> 4), LONG_MIN, LONG_MAX );
	}

	// To get better resolution in the gain parameters
	output = output >> pid->downscale;	// Here we shift instead of dividing for computational cost reasons

	/*
	 * Here we do the abs() and limit() functionality and save which way the saturation is happening
	 */
	if( output < -((int32_t)pid->max_output) ){
		pid->saturation = SATURATION_BOTTOM;
		output = -((int32_t)pid->max_output);
	}
	else if( output > ((int32_t)pid->max_output) ){
		pid->saturation = SATURATION_TOP;
		output = ((int32_t)pid->max_output);
	}
	else{
		pid->saturation = SATURATION_NONE;
	}
	return output;
}

#endif /* PID_H_ */
