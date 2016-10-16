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

#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include <util/atomic.h>
#include <limits.h>

#include "motor.h"
#include "pid.h"
#include "circBuffer.h"


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

/*
 * The actual PID structure
 */
typedef struct{
	volatile Motor* motor;

	circBuffer target_buffer, actual_buffer;

	Pid pid, pid_vel;

	uint8_t initialized;
	uint8_t notified_initialized;

	uint8_t enable;
	uint8_t control_mode;
	uint8_t feedback_mode;
	uint8_t target_mode;
	uint8_t polarity;
	uint8_t stream_mode;

	int32_t command_vel;
	int32_t maximum_vel;
	int32_t maximum_acc;

	uint16_t maximum_pwm;
	uint8_t mixed_mode_state;

	int32_t actual_tick_diff;

	int32_t output;
	uint8_t output_direction;

} Controller;

void ctrlClearState(volatile Controller* controller){
	//	controller->desired_to_use = desired_tick;
	controller->output = 0;
	controller->output_direction = 0;
	controller->command_vel = 0;
	controller->mixed_mode_state = MIXED_MODE_STATE_OFF;

	pidClearState(&controller->pid);
	pidClearState(&controller->pid_vel);

	circBufferReset(&controller->target_buffer);
	circBufferReset(&controller->actual_buffer);
}

void ctrlInitState(volatile Controller* controller, volatile Motor* motor_, unsigned char *target_buffer_data, unsigned char target_buffer_data_size, unsigned char *actual_buffer_data, unsigned char actual_buffer_data_size){
	circBufferInit(&controller->target_buffer, target_buffer_data, target_buffer_data_size);
	circBufferInit(&controller->actual_buffer, actual_buffer_data, actual_buffer_data_size);

	controller->maximum_pwm = FULL_PWM;
	controller->pid.max_output = FULL_PWM;
	controller->pid_vel.max_output = FULL_PWM;

	ctrlClearState(controller);
	controller->actual_buffer.size = 1;	// Default time delta for velocity is 1

	controller->motor = motor_;
	controller->notified_initialized = 0;
	controller->initialized = 0;
	controller->enable = ENABLE_OFF;
	controller->control_mode = CONTROL_MODE_POS;
	controller->feedback_mode = FEEDBACK_MODE_POT;
	controller->polarity = POLARITY_REGULAR;
	controller->stream_mode = STREAM_MODE_OFF;
	ctrlClearState(controller);
}

void ctrlChangeFeedback(volatile Controller* controller, uint8_t new_pot_mode){
	controller->feedback_mode = new_pot_mode;
	ctrlClearState(controller);
}

void ctrlChangeControlMode(volatile Controller* controller, uint8_t ctrl_mode){
	controller->control_mode = ctrl_mode;
	if( ctrl_mode == CONTROL_MODE_POS ){
		controller->pid.max_output = controller->maximum_pwm;
	}
	else{
		controller->pid_vel.max_output = controller->maximum_pwm;
	}
	ctrlClearState(controller);
}


void ctrlCalculateOutput(volatile Controller* controller){
	int32_t actual_tick;
	// Here we decide which reference to use
	if( controller->feedback_mode == FEEDBACK_MODE_POT ){
		actual_tick = controller->motor->actual_pot;
	}
	else{
		actual_tick = controller->motor->actual_enc;
	}
	circBufferPutLong(&controller->actual_buffer, actual_tick);

	if( 	controller->target_buffer.length < 4 ||
			controller->actual_buffer.length != controller->actual_buffer.size ||
			controller->initialized == 0 ){
		controller->output = 0;
		return;
	}

	controller->actual_tick_diff = actual_tick - circBufferPeekFirstLong(&controller->actual_buffer);
	int32_t vel_control;
	int32_t desired_tick;

	/*
	 * In non-streaming mode we just take the most recent target
	 */
	if( controller->stream_mode == STREAM_MODE_OFF ){
		desired_tick = circBufferPeekLastLong(&controller->target_buffer);
	}
	/*
	 * In streaming mode we always move towards the next positions in the buffer
	 */
	else{
		int32_t last_actual = circBufferPeekLongAtIndex(&controller->actual_buffer, controller->actual_buffer.length-8);
		desired_tick = circBufferPeekFirstLong(&controller->target_buffer);

		while( controller->target_buffer.length > 4 ){
			// In this case we just passed the desired_tick position and should advance to the next one
			if( 	(actual_tick >= desired_tick && last_actual <= desired_tick) ||
					(actual_tick <= desired_tick && last_actual >= desired_tick) ){
				circBufferGetFirstLong(&controller->target_buffer);
				desired_tick = circBufferPeekFirstLong(&controller->target_buffer);
			}
			// In this case we need to move towards this desired tick target
			else if( 	(actual_tick > desired_tick && last_actual > desired_tick) ||
						(actual_tick < desired_tick && last_actual < desired_tick) ){
				break;
			}
		}
	}

	/*
	 * Here we determine how to calculate the PID
	 */
	if( controller->control_mode == CONTROL_MODE_VEL ){
		if( controller->maximum_vel > 0 ){
			if( desired_tick > 0 ){
				controller->command_vel += MIN(desired_tick-controller->command_vel, controller->maximum_acc);
			}
			else{
				controller->command_vel += MAX(desired_tick-controller->command_vel, -controller->maximum_acc);
			}
			controller->command_vel = LIMIT(controller->command_vel, -controller->maximum_vel, controller->maximum_vel);
			controller->output = pidCalculateOutput(&controller->pid_vel, controller->command_vel, controller->actual_tick_diff);
		}
		else{
			controller->output = pidCalculateOutput(&controller->pid_vel, desired_tick, controller->actual_tick_diff);
		}
	}
	else if( controller->control_mode == CONTROL_MODE_POS ){
		controller->output = pidCalculateOutput(&controller->pid, desired_tick, actual_tick);
	}
	else if( controller->control_mode == CONTROL_MODE_MIXED ){
		// If our last output was positive
		if( controller->output >= 0 ){
			controller->command_vel += MIN(controller->maximum_vel-controller->command_vel, controller->maximum_acc);
		}
		else{
			controller->command_vel += MAX(-controller->maximum_vel-controller->command_vel, -controller->maximum_acc);
		}
		// Set the limit on the output as the commanded velocity (so it doesn't accumulate integrator while acceleration or velocity capped)
		controller->pid.max_output = ABS(controller->command_vel);

		vel_control = pidCalculateOutput(&controller->pid, desired_tick, actual_tick);
		controller->output = pidCalculateOutput(&controller->pid_vel, vel_control, controller->actual_tick_diff);
	}

	/*
	 * If we are streaming then we also go slower when going towards intermediate targets
	 */
	if( controller->stream_mode == STREAM_MODE_ON ){
		controller->output = controller->output >> ( controller->target_buffer.size-controller->target_buffer.length );
	}

	/*
	 * If the encoder phases are flipped with respect to the motor phases, this parameter will take care of that
	 */
	if( controller->polarity == POLARITY_FLIPPED ) controller->output = -(controller->output);

	if( controller->output < 0 ){
		controller->output = -(controller->output);
		controller->output_direction = OUTPUT_DIRECTION_CCW;
	}
	else if( controller->output > 0 ){
		controller->output_direction = OUTPUT_DIRECTION_CW;
	}
	else{
		controller->output_direction = OUTPUT_DIRECTION_NONE;
	}
}

#endif /* PID_H_ */
