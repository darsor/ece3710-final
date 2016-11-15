#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/*
	PB.5 - motor 1 IN1
	PB.7 - motor 1 IN2
	PB.6 - motor 1 PWM
	PB.2 - motor 2 IN1
	PB.3 - motor 2 IN2
	PB.4 - motor 2 PWM
*/

void motors_init(uint16_t freq);

// set motor speed (duty cycle), from -1.0 to 1.0
void motor1_speed(float speed);
void motor2_speed(float speed);

// set motor PWM frequency, in Hz
void motor1_freq(uint16_t freq);
void motor2_freq(uint16_t freq);

#endif