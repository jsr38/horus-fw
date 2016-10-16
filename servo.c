/*
  servo.c - servo motor driver: executes motion plans using servo motors
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

#include "system.h"
#include "nuts_bolts.h"
#include "servo.h"
#include "servo_pid.h"
#include "dc_motor.h"
#include "settings.h"
#include "planner.h"
#include "probe.h"


// Some useful constants.
#define DT_SEGMENT (1.0/(ACCELERATION_TICKS_PER_SECOND*60.0)) // min/segment 
#define req_deg_increment_SCALAR 1.25                                   
#define RAMP_ACCEL 0
#define RAMP_CRUISE 1
#define RAMP_DECEL 2

// Define Adaptive Multi-Axis Step-Smoothing(AMASS) levels and cutoff frequencies. The highest level
// frequency bin starts at 0Hz and ends at its cutoff frequency. The next lower level frequency bin
// starts at the next higher cutoff frequency, and so on. The cutoff frequencies for each level must
// be considered carefully against how much it over-drives the servo ISR, the accuracy of the 16-bit
// timer, and the CPU overhead. Level 0 (no AMASS, normal operation) frequency bin starts at the 
// Level 1 cutoff frequency and up to as fast as the CPU allows (over 30kHz in limited testing).
// NOTE: AMASS cutoff frequency multiplied by ISR overdrive factor must not exceed maximum step frequency.
// NOTE: Current settings are set to overdrive the ISR to no more than 16kHz, balancing CPU overhead
// and timer accuracy.  Do not alter these settings unless you know what you are doing.
#define MAX_AMASS_LEVEL 3
// AMASS_LEVEL0: Normal operation. No AMASS. No upper cutoff frequency. Starts at LEVEL1 cutoff frequency.
#define AMASS_LEVEL1 (F_CPU/8000) // Over-drives ISR (x2). Defined as F_CPU/(Cutoff frequency in Hz)
#define AMASS_LEVEL2 (F_CPU/4000) // Over-drives ISR (x4)
#define AMASS_LEVEL3 (F_CPU/2000) // Over-drives ISR (x8)


/*
 * Servo pin functionality
 */
#define SERVO_LONG_WAIT_TIME    190 // Defines long wait to be approx 15ms
#define SERVO_SHORT_WAIT_TIME   77 // Defines short wait to be approx 0.5ms

#define SERVO_TIMER_ENABLE 			TIMSK2 = (1 << OCIE2A); TCCR2A = (1 << WGM21); TCNT2 = 0; servo_state = 0; SERVO_SET_LONG_WAIT
#define SERVO_TIMER_DISABLE 		TIMSK2 = 0
#define SERVO_PRESCALER_1024		TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20)
#define SERVO_PRESCALER_256			TCCR2B = (1 << CS22) | (1 << CS21) | (0 << CS20)
#define SERVO_PRESCALER_128			TCCR2B = (1 << CS22) | (0 << CS21) | (1 << CS20)
#define SERVO_PRESCALER_64			TCCR2B = (1 << CS22) | (0 << CS21) | (0 << CS20)
#define SERVO_SET_SHORT_WAIT		SERVO_PRESCALER_256; OCR2A = SERVO_SHORT_WAIT_TIME // Set prescaler to 256 and timer to short
#define SERVO_SET_LONG_WAIT			SERVO_PRESCALER_1024; OCR2A = SERVO_LONG_WAIT_TIME // Set prescaler to 1024 and timer to long

volatile unsigned char servo_state;
volatile unsigned char servo_active;

/*
 * Global variables for both motors and their defaults
 */
uint8_t volatile id;
uint8_t volatile new_id;
volatile uint8_t should_change_id;

uint32_t volatile loop_count = 0;

uint8_t volatile pid_update_period = 195;	// Sets the update rate at around 100Hz

/*
 * Settings variables for the motors
 */

#define TARGET_BUFFER_NR_LONGS 		3
#define ACTUAL_BUFFER_NR_LONGS 		5

#define TARGET_BUFFER_SIZE 			(TARGET_BUFFER_NR_LONGS << 2)
#define ACTUAL_BUFFER_SIZE 			(ACTUAL_BUFFER_NR_LONGS << 2)
static uint8_t target_buffer_data_A[TARGET_BUFFER_SIZE];
static uint8_t target_buffer_data_B[TARGET_BUFFER_SIZE];
static uint8_t actual_buffer_data_A[ACTUAL_BUFFER_SIZE];
static uint8_t actual_buffer_data_B[ACTUAL_BUFFER_SIZE];

/*
 * a2d vars
 */
#define A2D_ITERATIONS_DIV2 	2
#define A2D_ITERATIONS 			1<<A2D_ITERATIONS_DIV2		// Needs to be divisible by 2

uint8_t volatile a2d_index;
uint8_t volatile a2d_counter;

uint16_t volatile a2d_value;
uint8_t volatile a2d_value_ready_flag;


// Stores the planner block Bresenham algorithm execution data for the segments in the segment 
// buffer. Normally, this buffer is partially in-use, but, for the worst case scenario, it will
// never exceed the number of accessible servo buffer segments (SEGMENT_BUFFER_SIZE-1).
// NOTE: This data is copied from the prepped planner blocks so that the planner blocks may be
// discarded when entirely consumed and completed by the segment buffer. Also, AMASS alters this
// data for its own use. 
typedef struct {  
  uint8_t direction_bits;
  uint32_t steps[N_AXIS];
  uint32_t step_event_count;
} servo_block_t;
static servo_block_t servo_block_buffer[SEGMENT_BUFFER_SIZE-1];

// Primary servo segment ring buffer. Contains small, short line segments for the servo 
// algorithm to execute, which are "checked-out" incrementally from the first block in the
// planner buffer. Once "checked-out", the steps in the segments buffer cannot be modified by 
// the planner, where the remaining planner block steps still can.
typedef struct {
  uint16_t n_step;          // Number of step events to be executed for this segment
  uint8_t servo_block_index;   // Servo block data index. Uses this information to execute this segment.
  uint16_t cycles_per_tick; // Step distance traveled per ISR tick, aka step rate.
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint8_t amass_level;    // Indicates AMASS level for the ISR to execute this segment
  #else
    uint8_t prescaler;      // Without AMASS, a prescaler is required to adjust for slow timing.
  #endif
} segment_t;
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

volatile motor_t motor;

// Servo ISR data struct. Contains the running data for the main servo ISR.
typedef struct {
  // Used by the bresenham line algorithm
  uint32_t counter_x,        // Counter variables for the bresenham line tracer
           counter_y, 
           counter_z;
  #ifdef STEP_PULSE_DELAY
    uint8_t step_bits;  // Stores out_bits output to complete the step pulse delay
  #endif
  
  uint8_t execute_step;     // Flags step execution for each interrupt.
  uint8_t step_pulse_time;  // Step pulse reset time after step rise
  uint8_t step_outbits;     // The next stepping-bits to be output
  uint8_t dir_outbits;
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint32_t steps[N_AXIS];
  #endif

  uint16_t step_count;       // Steps remaining in line segment motion  
  uint8_t exec_block_index; // Tracks the current servo_block index. Change indicates new block.
  servo_block_t *exec_block;   // Pointer to the block data for the segment being executed
  segment_t *exec_segment;  // Pointer to the segment being executed

  volatile motor_t* motor;

  //circBuffer target_buffer, actual_buffer;
  
  pid_t pid, pid_vel;
  
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
} servo_t;
static servo_t servo;

// Step segment ring buffer indices
static volatile uint8_t segment_buffer_tail;
static uint8_t segment_buffer_head;
static uint8_t segment_next_head;

// Step and direction port invert masks. 
static uint8_t step_port_invert_mask;
static uint8_t dir_port_invert_mask;

// Used to avoid ISR nesting of the "Servo Driver Interrupt". Should never occur though.
static volatile uint8_t busy;
static volatile uint8_t disable_motor;

// Pointers for the step segment being prepped from the planner buffer. Accessed only by the
// main program. Pointers may be planning segments or planner blocks ahead of what being executed.
static plan_block_t *pl_block;     // Pointer to the planner block being prepped
static servo_block_t *servo_prep_block;  // Pointer to the servo block data being prepped 

// Segment preparation data struct. Contains all the necessary information to compute new segments
// based on the current executing planner block.
typedef struct {
  uint8_t servo_block_index;  // Index of servo common data block being prepped
  uint8_t flag_partial_block;  // Flag indicating the last block completed. Time to load a new one.

  float steps_remaining;
  float step_per_deg;           // Current planner block step/deg conversion scalar
  float req_deg_increment;
  float dt_remainder;
  
  uint8_t ramp_type;      // Current segment ramp state
  float deg_complete;      // End of velocity profile from end of current planner block in (deg).
                          // NOTE: This value must coincide with a step(no mantissa) when converted.
  float current_speed;    // Current speed at the end of the segment buffer (deg/min)
  float maximum_speed;    // Maximum speed of executing block. Not always nominal speed. (deg/min)
  float exit_speed;       // Exit speed of executing block (deg/min)
  float accelerate_until; // Acceleration ramp end measured from end of block (deg)
  float decelerate_after; // Deceleration ramp start measured from end of block (deg)
} servo_prep_t;
static servo_prep_t prep;


/*    BLOCK VELOCITY PROFILE DEFINITION 
          __________________________
         /|                        |\     _________________         ^
        / |                        | \   /|               |\        |
       /  |                        |  \ / |               | \       s
      /   |                        |   |  |               |  \      p
     /    |                        |   |  |               |   \     e
    +-----+------------------------+---+--+---------------+----+    e
    |               BLOCK 1            ^      BLOCK 2          |    d
                                       |
                  time ----->      EXAMPLE: Block 2 entry speed is at max junction velocity
  
  The planner block buffer is planned assuming constant acceleration velocity profiles and are
  continuously joined at block junctions as shown above. However, the planner only actively computes
  the block entry speeds for an optimal velocity plan, but does not compute the block internal
  velocity profiles. These velocity profiles are computed ad-hoc as they are executed by the 
  servo algorithm and consists of only 7 possible types of profiles: cruise-only, cruise-
  deceleration, acceleration-cruise, acceleration-only, deceleration-only, full-trapezoid, and 
  triangle(no cruise).

                                        maximum_speed (< nominal_speed) ->  + 
                    +--------+ <- maximum_speed (= nominal_speed)          /|\                                         
                   /          \                                           / | \                      
 current_speed -> +            \                                         /  |  + <- exit_speed
                  |             + <- exit_speed                         /   |  |                       
                  +-------------+                     current_speed -> +----+--+                   
                   time -->  ^  ^                                           ^  ^                       
                             |  |                                           |  |                       
                decelerate_after(in deg)                            decelerate_after(in deg)
                    ^           ^                                           ^  ^
                    |           |                                           |  |
                accelerate_until(in deg)                            accelerate_until(in deg)
                    
  The step segment buffer computes the executing block velocity profile and tracks the critical
  parameters for the servo algorithm to accurately trace the profile. These critical parameters 
  are shown and defined in the above illustration.
*/


// Servo state initialization. Cycle should only start if the servo.cycle_start flag is
// enabled. Startup init and limits call this function but shouldn't start the cycle.
void servo_wake_up() 
{
  // Enable servo drivers.

  if (sys.state & (STATE_CYCLE | STATE_HOMING)){
    // Initialize servo output bits
    servo.dir_outbits = dir_port_invert_mask; 
    servo.step_outbits = step_port_invert_mask;
    
    // Enable Servo Driver Interrupt

  }
}

void servo_disable_on_idle(uint8_t disable)
{
  disable_motor = disable;
}

// Servo shutdown
void servo_go_idle() 
{
  busy = false;
  
  // Set servo driver idle state, disabled or enabled, depending on settings and circumstances.
  bool pin_state = false; // Keep enabled.
  if ((bit_istrue(sys.execute,EXEC_ALARM)) && sys.state != STATE_HOMING) {
    // Force servo dwell to lock axes for a defined amount of time to ensure the axes come to a complete
    pin_state = true; // Override. Disable servos.
  }
  if (disable_motor) {

  }
}

/* "The Servo Driver Interrupt" - */
ISR(TIMER1_COMPA_vect)
{        
  if (busy) { return; } // The busy-flag is used to avoid reentering this interrupt
  
  busy = true;
  sei(); // Re-enable interrupts to allow Servo Port Reset Interrupt to fire on-time. 
         // NOTE: The remaining code in this ISR will finish before returning to main program.
    
  // If there is no step segment, attempt to pop one from the servo buffer
  if (servo.exec_segment == NULL) {
    // Anything in the buffer? If so, load and initialize next step segment.
    if (segment_buffer_head != segment_buffer_tail) {
      // Initialize new step segment and load number of steps to execute
      servo.exec_segment = &segment_buffer[segment_buffer_tail];

      #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        // With AMASS is disabled, set timer prescaler for segments with slow step frequencies (< 250Hz).
        TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (servo.exec_segment->prescaler<<CS10);
      #endif

      // Initialize step segment timing per step and load number of steps to execute.
      OCR1A = servo.exec_segment->cycles_per_tick;
      servo.step_count = servo.exec_segment->n_step; // NOTE: Can sometimes be zero when moving slow.
      // If the new segment starts a new planner block, initialize servo variables and counters.
      // NOTE: When the segment data index changes, this indicates a new planner block.
      if ( servo.exec_block_index != servo.exec_segment->servo_block_index ) {
        servo.exec_block_index = servo.exec_segment->servo_block_index;
        servo.exec_block = &servo_block_buffer[servo.exec_block_index];
        
        // Initialize Bresenham line and distance counters
        servo.counter_x = (servo.exec_block->step_event_count >> 1);
        servo.counter_y = servo.counter_x;
        servo.counter_z = servo.counter_x;        
      }

      servo.dir_outbits = servo.exec_block->direction_bits ^ dir_port_invert_mask; 

      #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        // With AMASS enabled, adjust Bresenham axis increment counters according to AMASS level.
        servo.steps[X_AXIS] = servo.exec_block->steps[X_AXIS] >> servo.exec_segment->amass_level;
        servo.steps[Y_AXIS] = servo.exec_block->steps[Y_AXIS] >> servo.exec_segment->amass_level;
        servo.steps[Z_AXIS] = servo.exec_block->steps[Z_AXIS] >> servo.exec_segment->amass_level;
      #endif
      
    } else {
      // Segment buffer empty. Shutdown.
      servo_go_idle();
      bit_true_atomic(sys.execute,EXEC_CYCLE_STOP); // Flag main program for cycle end
      return; // Nothing to do but exit.
    }  
  }
  
  
  // Check probing state.
  probe_state_monitor();
   
  // Reset step out bits.
  servo.step_outbits = 0; 


  // During a homing cycle, lock out and prevent desired axes from moving.
  if (sys.state == STATE_HOMING) { /* servo.step_outbits &= sys.homing_axis_lock;*/ }   

  servo.step_count--; // Decrement step events count 
  if (servo.step_count == 0) {
    // Segment is complete. Discard current segment and advance segment indexing.
    servo.exec_segment = NULL;
    if ( ++segment_buffer_tail == SEGMENT_BUFFER_SIZE) { segment_buffer_tail = 0; }
  }

  servo.step_outbits ^= step_port_invert_mask;  // Apply step port invert mask    
  busy = false;

}

    st_update_plan_block_parameters();
// Generates the step and direction port invert masks used in the Servo Interrupt Driver.
void servo_generate_step_dir_invert_masks()
{
  /*
  uint8_t idx;
  step_port_invert_mask = 0;
  dir_port_invert_mask = 0;
  for (idx=0; idx<N_AXIS; idx++) {
    if (bit_istrue(settings.step_invert_mask,bit(idx))) { step_port_invert_mask |= get_step_pin_mask(idx); }
    if (bit_istrue(settings.dir_invert_mask,bit(idx))) { dir_port_invert_mask |= get_direction_pin_mask(idx); }
  }
  */
}


// Reset and clear servo subsystem variables
void servo_reset()
{
  disable_motor = true;

  // Initialize servo driver idle state.
  servo_go_idle();
  
  // Initialize servo algorithm variables.
  memset(&prep, 0, sizeof(prep));
  memset(&servo, 0, sizeof(servo));
  servo.exec_segment = NULL;
  pl_block = NULL;  // Planner block pointer used by segment buffer
  segment_buffer_tail = 0;
  segment_buffer_head = 0; // empty = tail
  segment_next_head = 1;
  busy = false;
  
  servo_generate_step_dir_invert_masks();
      
  // Initialize step and direction port pins.
  //  STEP_PORT = (STEP_PORT & ~STEP_MASK) | step_port_invert_mask;
  //  DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | dir_port_invert_mask;

  //	servo.desired_to_use = desired_tick;
  servo.output = 0;
  servo.output_direction = 0;
  servo.command_vel = 0;
  servo.mixed_mode_state = MIXED_MODE_STATE_OFF;

  pid_clear_state(&servo.pid);
  pid_clear_state(&servo.pid_vel);

  // circBufferReset(&servo.target_buffer);  CIRC
  // circBufferReset(&servo.actual_buffer);  CIRC

}


// Initialize and start the servo motor subsystem
void servo_init()
{
  // Configure step and direction interface pins
  //  STEP_DDR |= STEP_MASK;
  // SERVOS_DISABLE_DDR |= 1<<SERVOS_DISABLE_BIT;
  // DIRECTION_DDR |= DIRECTION_MASK;

  // Configure Timer 1: Servo Driver Interrupt
  TCCR1B &= ~(1<<WGM13); // waveform generation = 0100 = CTC
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~((1<<WGM11) | (1<<WGM10)); 
  TCCR1A &= ~((1<<COM1A1) | (1<<COM1A0) | (1<<COM1B1) | (1<<COM1B0)); // Disconnect OC1 output
  // TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10); // Set in servo_go_idle().
  // TIMSK1 &= ~(1<<OCIE1A);  // Set in servo_go_idle().
  
  // Configure Timer 0: Servo Port Reset Interrupt
  TIMSK0 &= ~((1<<OCIE0B) | (1<<OCIE0A) | (1<<TOIE0)); // Disconnect OC0 outputs and OVF interrupt.
  TCCR0A = 0; // Normal operation
  TCCR0B = 0; // Disable Timer0 until needed
  TIMSK0 |= (1<<TOIE0); // Enable Timer0 overflow interrupt
  #ifdef STEP_PULSE_DELAY
    TIMSK0 |= (1<<OCIE0A); // Enable Timer0 Compare Match A interrupt
  #endif

    motor_init(&motor, 0);
    
    // circBufferInit(&servo.target_buffer, target_buffer_data, target_buffer_data_size);  CIRC
    // circBufferInit(&servo.actual_buffer, actual_buffer_data, actual_buffer_data_size);  CIRC
    
    servo.maximum_pwm = FULL_PWM;
    servo.pid.max_output = FULL_PWM;
    servo.pid_vel.max_output = FULL_PWM;
    
    //    ctrlClearState(servo);
    //    servo.actual_buffer.size = 1;	// Default time delta for velocity is 1
    servo.motor = &motor;
    servo.notified_initialized = 0;
    servo.initialized = 0;
    servo.enable = ENABLE_OFF;
    servo.control_mode = CONTROL_MODE_POS;
    servo.feedback_mode = FEEDBACK_MODE_POT;
    servo.polarity = POLARITY_REGULAR;
    servo.stream_mode = STREAM_MODE_OFF;
    servo_reset();
	
}

void servo_set_feedback(uint8_t new_pot_mode){
  servo.feedback_mode = new_pot_mode;
  servo_reset();
}

void servo_set_control_mode(uint8_t ctrl_mode){
  servo.control_mode = ctrl_mode;
  if( ctrl_mode == CONTROL_MODE_POS ) {
    servo.pid.max_output = servo.maximum_pwm;
  }
  else{
    servo.pid_vel.max_output = servo.maximum_pwm;
  }
  servo_reset();
}


// Called by planner_recalculate() when the executing block is updated by the new plan.
void servo_update_plan_block_parameters()
{ 
  if (pl_block != NULL) { // Ignore if at start of a new block.
    prep.flag_partial_block = true;
    pl_block->entry_speed_sqr = prep.current_speed*prep.current_speed; // Update entry speed.
    pl_block = NULL; // Flag servo_prep_segment() to load new velocity profile.
  }
}


/* Prepares step segment buffer. Continuously called from main program. 

   The segment buffer is an intermediary buffer interface between the execution of steps
   by the servo algorithm and the velocity profiles generated by the planner. The servo
   algorithm only executes steps within the segment buffer and is filled by the main program
   when steps are "checked-out" from the first block in the planner buffer. This keeps the
   step execution and planning optimization processes atomic and protected from each other.
   The number of steps "checked-out" from the planner buffer and the number of segments in
   the segment buffer is sized and computed such that no operation in the main program takes
   longer than the time it takes the servo algorithm to empty it before refilling it. 
   Currently, the segment buffer conservatively holds roughly up to 40-50 msec of steps.
   NOTE: Computation units are in steps, degrees, and minutes.
*/
void servo_prep_buffer()
{
  while (segment_buffer_tail != segment_next_head) { // Check if we need to fill the buffer.

    // Determine if we need to load a new planner block or if the block has been replanned. 
    if (pl_block == NULL) {
      pl_block = plan_get_current_block(); // Query planner for a queued block
      if (pl_block == NULL) { return; } // No planner blocks. Exit.
                      
      // Check if the segment buffer completed the last planner block. If so, load the Bresenham
      // data for the block. If not, we are still mid-block and the velocity profile was updated. 
      if (prep.flag_partial_block) {
        prep.flag_partial_block = false; // Reset flag
      } else {
        // Increment servo common data index to store new planner block data. 
        if ( ++prep.servo_block_index == (SEGMENT_BUFFER_SIZE-1) ) { prep.servo_block_index = 0; }
        
        // Prepare and copy Bresenham algorithm segment data from the new planner block, so that
        // when the segment buffer completes the planner block, it may be discarded when the 
        // segment buffer finishes the prepped block, but the servo ISR is still executing it. 
        servo_prep_block = &servo_block_buffer[prep.servo_block_index];
        servo_prep_block->direction_bits = pl_block->direction_bits;
        #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
          servo_prep_block->steps[X_AXIS] = pl_block->steps[X_AXIS];
          servo_prep_block->steps[Y_AXIS] = pl_block->steps[Y_AXIS];
          servo_prep_block->steps[Z_AXIS] = pl_block->steps[Z_AXIS];
          servo_prep_block->step_event_count = pl_block->step_event_count;
        #else
          // With AMASS enabled, simply bit-shift multiply all Bresenham data by the max AMASS 
          // level, such that we never divide beyond the original data anywhere in the algorithm.
          // If the original data is divided, we can lose a step from integer roundoff.
          servo_prep_block->steps[X_AXIS] = pl_block->steps[X_AXIS] << MAX_AMASS_LEVEL;
          servo_prep_block->steps[Y_AXIS] = pl_block->steps[Y_AXIS] << MAX_AMASS_LEVEL;
          servo_prep_block->steps[Z_AXIS] = pl_block->steps[Z_AXIS] << MAX_AMASS_LEVEL;
          servo_prep_block->step_event_count = pl_block->step_event_count << MAX_AMASS_LEVEL;
        #endif
        
        // Initialize segment buffer data for generating the segments.
        prep.steps_remaining = pl_block->step_event_count;
        prep.step_per_deg = prep.steps_remaining/pl_block->degrees;
        prep.req_deg_increment = req_deg_increment_SCALAR/prep.step_per_deg;
        
        prep.dt_remainder = 0.0; // Reset for new planner block

        if (sys.state == STATE_HOLD) {
          // Override planner block entry speed and enforce deceleration during feed hold.
          prep.current_speed = prep.exit_speed; 
          pl_block->entry_speed_sqr = prep.exit_speed*prep.exit_speed; 
        }
        else { prep.current_speed = sqrt(pl_block->entry_speed_sqr); }
      }
     
      /* --------------------------------------------------------------------------------- 
         Compute the velocity profile of a new planner block based on its entry and exit
         speeds, or recompute the profile of a partially-completed planner block if the 
         planner has updated it. For a commanded forced-deceleration, such as from a feed 
         hold, override the planner velocities and decelerate to the target exit speed.
      */
      prep.deg_complete = 0.0; // Default velocity profile complete at 0.0deg from end of block.
      float inv_2_accel = 0.5/pl_block->acceleration;
      if (sys.state == STATE_HOLD) { // [Forced Deceleration to Zero Velocity]
        // Compute velocity profile parameters for a feed hold in-progress. This profile overrides
        // the planner block profile, enforcing a deceleration to zero speed.
        prep.ramp_type = RAMP_DECEL;
        // Compute decelerate distance relative to end of block.
        float decel_dist = pl_block->degrees - inv_2_accel*pl_block->entry_speed_sqr;
        if (decel_dist < 0.0) {
          // Deceleration through entire planner block. End of feed hold is not in this block.
          prep.exit_speed = sqrt(pl_block->entry_speed_sqr-2*pl_block->acceleration*pl_block->degrees);
        } else {
          prep.deg_complete = decel_dist; // End of feed hold.
          prep.exit_speed = 0.0;
        }
      } else { // [Normal Operation]
        // Compute or recompute velocity profile parameters of the prepped planner block.
        prep.ramp_type = RAMP_ACCEL; // Initialize as acceleration ramp.
        prep.accelerate_until = pl_block->degrees; 
        prep.exit_speed = plan_get_exec_block_exit_speed();   
        float exit_speed_sqr = prep.exit_speed*prep.exit_speed;
        float intersect_distance =
                0.5*(pl_block->degrees+inv_2_accel*(pl_block->entry_speed_sqr-exit_speed_sqr));
        if (intersect_distance > 0.0) {
          if (intersect_distance < pl_block->degrees) { // Either trapezoid or triangle types
            // NOTE: For acceleration-cruise and cruise-only types, following calculation will be 0.0.
            prep.decelerate_after = inv_2_accel*(pl_block->nominal_speed_sqr-exit_speed_sqr);
            if (prep.decelerate_after < intersect_distance) { // Trapezoid type
              prep.maximum_speed = sqrt(pl_block->nominal_speed_sqr);
              if (pl_block->entry_speed_sqr == pl_block->nominal_speed_sqr) { 
                // Cruise-deceleration or cruise-only type.
                prep.ramp_type = RAMP_CRUISE;
              } else {
                // Full-trapezoid or acceleration-cruise types
                prep.accelerate_until -= inv_2_accel*(pl_block->nominal_speed_sqr-pl_block->entry_speed_sqr); 
              }
            } else { // Triangle type
              prep.accelerate_until = intersect_distance;
              prep.decelerate_after = intersect_distance;
              prep.maximum_speed = sqrt(2.0*pl_block->acceleration*intersect_distance+exit_speed_sqr);
            }          
          } else { // Deceleration-only type
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->degrees;
            prep.maximum_speed = prep.current_speed;
          }
        } else { // Acceleration-only type
          prep.accelerate_until = 0.0;
          // prep.decelerate_after = 0.0;
          prep.maximum_speed = prep.exit_speed;
        }
      }  
    }

    // Initialize new segment
    segment_t *prep_segment = &segment_buffer[segment_buffer_head];

    // Set new segment to point to the current segment data block.
    prep_segment->servo_block_index = prep.servo_block_index;

    /*------------------------------------------------------------------------------------
        Compute the average velocity of this new segment by determining the total distance
      traveled over the segment time DT_SEGMENT. The following code first attempts to create 
      a full segment based on the current ramp conditions. If the segment time is incomplete 
      when terminating at a ramp state change, the code will continue to loop through the
      progressing ramp states to fill the remaining segment execution time. However, if 
      an incomplete segment terminates at the end of the velocity profile, the segment is 
      considered completed despite having a truncated execution time less than DT_SEGMENT.
        The velocity profile is always assumed to progress through the ramp sequence:
      acceleration ramp, cruising state, and deceleration ramp. Each ramp's travel distance
      may range from zero to the length of the block. Velocity profiles can end either at 
      the end of planner block (typical) or mid-block at the end of a forced deceleration, 
      such as from a feed hold.
    */
    float dt_max = DT_SEGMENT; // Maximum segment time
    float dt = 0.0; // Initialize segment time
    float time_var = dt_max; // Time worker variable
    float deg_var; // deg-Distance worker variable
    float speed_var; // Speed worker variable   
    float deg_remaining = pl_block->degrees; // New segment distance from end of block.
    float minimum_deg = deg_remaining-prep.req_deg_increment; // Guarantee at least one step.
    if (minimum_deg < 0.0) { minimum_deg = 0.0; }

    do {
      switch (prep.ramp_type) {
        case RAMP_ACCEL: 
          // NOTE: Acceleration ramp only computes during first do-while loop.
          speed_var = pl_block->acceleration*time_var;
          deg_remaining -= time_var*(prep.current_speed + 0.5*speed_var);
          if (deg_remaining < prep.accelerate_until) { // End of acceleration ramp.
            // Acceleration-cruise, acceleration-deceleration ramp junction, or end of block.
            deg_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB
            time_var = 2.0*(pl_block->degrees-deg_remaining)/(prep.current_speed+prep.maximum_speed);
            if (deg_remaining == prep.decelerate_after) { prep.ramp_type = RAMP_DECEL; }
            else { prep.ramp_type = RAMP_CRUISE; }
            prep.current_speed = prep.maximum_speed;
          } else { // Acceleration only. 
            prep.current_speed += speed_var;
          }
          break;
        case RAMP_CRUISE: 
          // NOTE: deg_var used to retain the last deg_remaining for incomplete segment time_var calculations.
          // NOTE: If maximum_speed*time_var value is too low, round-off can cause deg_var to not change. To 
          //   prevent this, simply enforce a minimum speed threshold in the planner.
          deg_var = deg_remaining - prep.maximum_speed*time_var;
          if (deg_var < prep.decelerate_after) { // End of cruise. 
            // Cruise-deceleration junction or end of block.
            time_var = (deg_remaining - prep.decelerate_after)/prep.maximum_speed;
            deg_remaining = prep.decelerate_after; // NOTE: 0.0 at EOB
            prep.ramp_type = RAMP_DECEL;
          } else { // Cruising only.         
            deg_remaining = deg_var; 
          } 
          break;
        default: // case RAMP_DECEL:
          // NOTE: deg_var used as a misc worker variable to prevent errors when near zero speed.
          speed_var = pl_block->acceleration*time_var; // Used as delta speed (deg/min)
          if (prep.current_speed > speed_var) { // Check if at or below zero speed.
            // Compute distance from end of segment to end of block.
            deg_var = deg_remaining - time_var*(prep.current_speed - 0.5*speed_var); // (deg)
            if (deg_var > prep.deg_complete) { // Deceleration only.
              deg_remaining = deg_var;
              prep.current_speed -= speed_var;
              break; // Segment complete. Exit switch-case statement. Continue do-while loop.
            }
          } // End of block or end of forced-deceleration.
          time_var = 2.0*(deg_remaining-prep.deg_complete)/(prep.current_speed+prep.exit_speed);
          deg_remaining = prep.deg_complete; 
      }
      dt += time_var; // Add computed ramp time to total segment time.
      if (dt < dt_max) { time_var = dt_max - dt; } // **Incomplete** At ramp junction.
      else {
        if (deg_remaining > minimum_deg) { // Check for very slow segments with zero steps.
          // Increase segment time to ensure at least one step in segment. Override and loop
          // through distance calculations until minimum_deg or deg_complete.
          dt_max += DT_SEGMENT;
          time_var = dt_max - dt;
        } else { 
          break; // **Complete** Exit loop. Segment execution time maxed.
        }
      }
    } while (deg_remaining > prep.deg_complete); // **Complete** Exit loop. Profile complete.

   
    /* -----------------------------------------------------------------------------------
       Compute segment step rate, steps to execute, and apply necessary rate corrections.
       NOTE: Steps are computed by direct scalar conversion of the millimeter distance 
       remaining in the block, rather than incrementally tallying the steps executed per
       segment. This helps in removing floating point round-off issues of several additions. 
       However, since floats have only 7.2 significant digits, long moves with extremely 
       high step counts can exceed the precision of floats, which can lead to lost steps.
       Fortunately, this scenario is highly unlikely and unrealistic in CNC machines
       supported by Grbl (i.e. exceeding 10 meters axis travel at 200 step/deg).
    */
    float steps_remaining = prep.step_per_deg*deg_remaining; // Convert deg_remaining to steps
    float n_steps_remaining = ceil(steps_remaining); // Round-up current steps remaining
    float laservo_n_steps_remaining = ceil(prep.steps_remaining); // Round-up last steps remaining
    prep_segment->n_step = laservo_n_steps_remaining-n_steps_remaining; // Compute number of steps to execute.
    
    // Bail if we are at the end of a feed hold and don't have a step to execute.
    if (prep_segment->n_step == 0) {
      if (sys.state == STATE_HOLD) {

        // Less than one step to decelerate to zero speed, but already very close. AMASS 
        // requires full steps to execute. So, just bail.
        prep.current_speed = 0.0;
        prep.dt_remainder = 0.0;
        prep.steps_remaining = n_steps_remaining;
        pl_block->degrees = prep.steps_remaining/prep.step_per_deg; // Update with full steps.
        plan_cycle_reinitialize();         
        sys.state = STATE_QUEUED; 
        return; // Segment not generated, but current step data still retained.
      }
    }

    // Compute segment step rate. Since steps are integers and deg distances traveled are not,
    // the end of every segment can have a partial step of varying magnitudes that are not 
    // executed, because the servo ISR requires whole steps due to the AMASS algorithm. To
    // compensate, we track the time to execute the previous segment's partial step and simply
    // apply it with the partial step distance to the current segment, so that it minutely
    // adjusts the whole segment rate to keep step output exact. These rate adjustments are 
    // typically very small and do not adversely effect performance, but ensures that Grbl
    // outputs the exact acceleration and velocity profiles as computed by the planner.
    dt += prep.dt_remainder; // Apply previous segment partial step execute time
    float inv_rate = dt/(laservo_n_steps_remaining - steps_remaining); // Compute adjusted step rate inverse
    prep.dt_remainder = (n_steps_remaining - steps_remaining)*inv_rate; // Update segment partial step time

    // Compute CPU cycles per step for the prepped segment.
    uint32_t cycles = ceil( (TICKS_PER_MICROSECOND*1000000*60)*inv_rate ); // (cycles/step)    

    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING        
      // Compute step timing and multi-axis smoothing level.
      // NOTE: AMASS overdrives the timer with each level, so only one prescalar is required.
      if (cycles < AMASS_LEVEL1) { prep_segment->amass_level = 0; }
      else {
        if (cycles < AMASS_LEVEL2) { prep_segment->amass_level = 1; }
        else if (cycles < AMASS_LEVEL3) { prep_segment->amass_level = 2; }
        else { prep_segment->amass_level = 3; }    
        cycles >>= prep_segment->amass_level; 
        prep_segment->n_step <<= prep_segment->amass_level;
      }
      if (cycles < (1UL << 16)) { prep_segment->cycles_per_tick = cycles; } // < 65536 (4.1ms @ 16MHz)
      else { prep_segment->cycles_per_tick = 0xffff; } // Just set the slowest speed possible.
    #else 
      // Compute step timing and timer prescalar for normal step generation.
      if (cycles < (1UL << 16)) { // < 65536  (4.1ms @ 16MHz)
        prep_segment->prescaler = 1; // prescaler: 0
        prep_segment->cycles_per_tick = cycles;
      } else if (cycles < (1UL << 19)) { // < 524288 (32.8ms@16MHz)
        prep_segment->prescaler = 2; // prescaler: 8
        prep_segment->cycles_per_tick = cycles >> 3;
      } else { 
        prep_segment->prescaler = 3; // prescaler: 64
        if (cycles < (1UL << 22)) { // < 4194304 (262ms@16MHz)
          prep_segment->cycles_per_tick =  cycles >> 6;
        } else { // Just set the slowest speed possible. (Around 4 step/sec.)
          prep_segment->cycles_per_tick = 0xffff;
        }
      }
    #endif

    // Segment complete! Increment segment buffer indices.
    segment_buffer_head = segment_next_head;
    if ( ++segment_next_head == SEGMENT_BUFFER_SIZE ) { segment_next_head = 0; }

    // Setup initial conditions for next segment.
    if (deg_remaining > prep.deg_complete) { 
      // Normal operation. Block incomplete. Distance remaining in block to be executed.
      pl_block->degrees = deg_remaining;      
      prep.steps_remaining = steps_remaining;  
    } else { 
      // End of planner block or forced-termination. No more distance to be executed.
      if (deg_remaining > 0.0) { // At end of forced-termination.
        // Reset prep parameters for resuming and then bail.
        // NOTE: Currently only feed holds qualify for this scenario. May change with overrides.       
        prep.current_speed = 0.0;
        prep.dt_remainder = 0.0;
        prep.steps_remaining = ceil(steps_remaining);
        pl_block->degrees = prep.steps_remaining/prep.step_per_deg; // Update with full steps.
        plan_cycle_reinitialize(); 
        sys.state = STATE_QUEUED; // End cycle.        

        return; // Bail!
// TODO: Try to move QUEUED setting into cycle re-initialize.

      } else { // End of planner block
        // The planner block is complete. All steps are set to be executed in the segment buffer.
        pl_block = NULL;
        plan_discard_current_block();
      }
    }

  } 
}      

void servo_calculate_output(){
  int32_t actual_tick;
  // Here we decide which reference to use
  if (servo.feedback_mode == FEEDBACK_MODE_POT) {
    actual_tick = servo.motor->actual_pot;
  }
  else {
    actual_tick = servo.motor->actual_enc;
  }
  //circBufferPutLong(&servo.actual_buffer, actual_tick);

  /*
  if (servo.target_buffer.length < 4 ||
      servo.actual_buffer.length != servo.actual_buffer.size ||
      servo.initialized == 0) {
    servo.output = 0;
    return;
  }
  */
  
  servo.actual_tick_diff = actual_tick /*- circBufferPeekFirstLong(&servo.actual_buffer)*/;
  int32_t vel_control;
  int32_t desired_tick;

  /*
   * In non-streaming mode we just take the most recent target
   */
  if (servo.stream_mode == STREAM_MODE_OFF) {
    desired_tick = 0; //circBufferPeekLastLong(&servo.target_buffer);
  }
  /*
   * In streaming mode we always move towards the next positions in the buffer
   */
  else {
    int32_t last_actual = 0; //circBufferPeekLongAtIndex(&servo.actual_buffer, servo.actual_buffer.length-8);
    desired_tick = 0; //circBufferPeekFirstLong(&servo.target_buffer);
	  
    //    while( servo.target_buffer.length > 4 ){
      // In this case we just passed the desired_tick position and should advance to the next one
      if( 	(actual_tick >= desired_tick && last_actual <= desired_tick) ||
		(actual_tick <= desired_tick && last_actual >= desired_tick) ){
	//circBufferGetFirstLong(&servo.target_buffer);
	desired_tick = 0; //circBufferPeekFirstLong(&servo.target_buffer);
      }
      // In this case we need to move towards this desired tick target
      else if( 	(actual_tick > desired_tick && last_actual > desired_tick) ||
		(actual_tick < desired_tick && last_actual < desired_tick) ){
	//break;
      }
      //}
  }

  /*
   * Here we determine how to calculate the PID
   */
  if( servo.control_mode == CONTROL_MODE_VEL ){
    if( servo.maximum_vel > 0 ){
      if( desired_tick > 0 ){
	servo.command_vel += MIN(desired_tick-servo.command_vel, servo.maximum_acc);
      }
      else{
	servo.command_vel += MAX(desired_tick-servo.command_vel, -servo.maximum_acc);
      }
      servo.command_vel = LIMIT(servo.command_vel, -servo.maximum_vel, servo.maximum_vel);
      servo.output = pid_calculate_output(&servo.pid_vel, servo.command_vel, servo.actual_tick_diff);
    }
    else{
      servo.output = pid_calculate_output(&servo.pid_vel, desired_tick, servo.actual_tick_diff);
    }
  }
  else if( servo.control_mode == CONTROL_MODE_POS ){
    servo.output = pid_calculate_output(&servo.pid, desired_tick, actual_tick);
  }
  else if( servo.control_mode == CONTROL_MODE_MIXED ){
    // If our last output was positive
    if( servo.output >= 0 ){
      servo.command_vel += MIN(servo.maximum_vel-servo.command_vel, servo.maximum_acc);
    }
    else{
      servo.command_vel += MAX(-servo.maximum_vel-servo.command_vel, -servo.maximum_acc);
    }
    // Set the limit on the output as the commanded velocity (so it doesn't accumulate integrator while acceleration or velocity capped)
    servo.pid.max_output = ABS(servo.command_vel);

    vel_control = pid_calculate_output(&servo.pid, desired_tick, actual_tick);
    servo.output = pid_calculate_output(&servo.pid_vel, vel_control, servo.actual_tick_diff);
  }

  /*
   * If we are streaming then we also go slower when going towards intermediate targets
   */
  if( servo.stream_mode == STREAM_MODE_ON ){
    //servo.output = servo.output >> ( servo.target_buffer.size-servo.target_buffer.length );
  }

  /*
   * If the encoder phases are flipped with respect to the motor phases, this parameter will take care of that
   */
  if( servo.polarity == POLARITY_FLIPPED ) servo.output = -(servo.output);

  if( servo.output < 0 ){
    servo.output = -(servo.output);
    servo.output_direction = OUTPUT_DIRECTION_CCW;
  }
  else if( servo.output > 0 ){
    servo.output_direction = OUTPUT_DIRECTION_CW;
  }
  else{
    servo.output_direction = OUTPUT_DIRECTION_NONE;
  }
}


// Called by runtime status reporting to fetch the current speed being executed. This value
// however is not exactly the current speed, but the speed computed in the last step segment
// in the segment buffer. It will always be behind by up to the number of segment blocks (-1)
// divided by the ACCELERATION TICKS PER SECOND in seconds. 
#ifdef REPORT_REALTIME_RATE
  float servo_get_realtime_rate()
  {
     if (sys.state & (STATE_CYCLE | STATE_HOMING | STATE_HOLD)){
       return prep.current_speed;
     }
    return 0.0f;
  }
#endif
