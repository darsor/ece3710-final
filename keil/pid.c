#include "pid.h"

uint8_t using_limits = 0;
uint8_t using_dampening = 0;
uint8_t using_deadzone = 0;
uint8_t using_int_range = 0;
uint8_t is_initialized = 0;

float error_old = 0;
float Kp = 0, Ki = 0, Kd = 0, dt = 1;
float upper_limit, lower_limit;
float upper_damp, lower_damp;
float upper_deadzone, lower_deadzone;
float upper_int_range, lower_int_range;
float integral = 0, derivative = 0, output = 0;

float update(float sp, float pv) {
	// calculate error (proportional)
	float error = sp - pv;
	if (!is_initialized) {
		error_old = error;
		is_initialized = 1;
	}
	
	// calculate integral
	if (!using_int_range || (error < upper_int_range && error > lower_int_range)) {
		integral += error * dt;
	}
	if (using_dampening && error < upper_damp && error > lower_damp) {
		integral = 0;
		error = 0; //makes dampening work with deadzone
	}
	
	// calculate derivative
	derivative = (error-error_old) / dt;
	
	// calculate output
	output = Kp*error + Ki*integral + Kd*derivative;
	
	// apply limits and deadzone
	if (using_limits) {
			 if (output > upper_limit) output = upper_limit;
		else if (output < lower_limit) output = lower_limit;
	}
	if (using_deadzone && error != 0) {
		if (error < 0) output += lower_deadzone;
		else output += upper_deadzone;	// constant offset
	}
	if (error == 0) output = 0;
	
	error_old = error;
	return output;
}

void initialize_pid(float p, float i, float d, float t) {
	Kp = p;
	Ki = i;
	Kd = d;
	dt = t;
}

void set_limits(float lower, float upper) {
	upper_limit = upper;
	lower_limit = lower;
	using_limits = 1;
}

void set_dampening(float low, float high) {
	upper_damp = high;
	lower_damp = low;
	using_dampening = 1;
}

void set_deadzone(float low, float high) {
	upper_deadzone = high;
	lower_deadzone = low;
	using_deadzone = 1;
}

void set_integral_range(float low, float high) {
	upper_int_range = high;
	lower_int_range = low;
	using_int_range = 1;
}
