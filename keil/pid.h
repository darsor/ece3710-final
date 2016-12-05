#ifndef PID_H
#define PID_H

#include <stdint.h>

// PID variables
extern uint8_t using_limits;
extern uint8_t using_dampening;
extern uint8_t using_deadzone;
extern uint8_t using_int_range;
extern uint8_t is_initialized;

extern float error_old;
extern float dt, Kp, Ki, Kd;
extern float upper_limit, lower_limit;
extern float upper_damp, lower_damp;
extern float upper_deadzone, lower_deadzone;
extern float upper_int_range, lower_int_range;
extern float integral, derivative, output;

// PID functions
float pid_update(float sp, float pv);
void initialize_pid(float p, float i, float d, float dt);
void set_limits(float lower, float upper);			// output will not surpass these limits
void set_dampening(float low, float high);         	// integral term will be damped in this range
void set_deadzone(float low, float high);          	// the "zero" value at which the system responds
void set_integral_range(float low, float high);     // the integral term will not build up outside of this range
void reset_i_term(void);

#endif
