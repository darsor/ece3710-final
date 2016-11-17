#include "motor.h"
#include "tiva_c.h"

/*
	PB.5 - motor 1 IN1
	PB.7 - motor 1 IN2
	PB.6 - motor 1 PWM
	PB.2 - motor 2 IN1
	PB.3 - motor 2 IN2
	PB.4 - motor 2 PWM
*/

void motors_init(uint32_t clk_speed, uint16_t freq) {
	uint8_t pins = 0xAC;
	gpio_init(GPIO_B, pins, GPIO_OUT, GPIO_DEN | GPIO_PUR);
	gpio_write(GPIO_B, 2, 1);
	gpio_write(GPIO_B, 4, 1);
	gpio_write(GPIO_B, 5, 1);
	gpio_write(GPIO_B, 7, 1);
	pwm_init(PWM0, clk_speed, freq);
	pwm_init(PWM1, clk_speed, freq);
}

void motor1_speed(float speed) {
	if (speed == 0) {
		gpio_write(GPIO_B, 5, 1);
		gpio_write(GPIO_B, 7, 1);
	} else if (speed > 0) {
		gpio_write(GPIO_B, 5, 1);
		gpio_write(GPIO_B, 7, 0);
	} else {
		gpio_write(GPIO_B, 7, 1);
		gpio_write(GPIO_B, 5, 0);
		speed *= -1;
	}
	pwm_set_duty(PWM0, speed);
}

void motor2_speed(float speed) {
	if (speed == 0) {
		gpio_write(GPIO_B, 2, 1);
		gpio_write(GPIO_B, 4, 1);
	} else if (speed > 0) {
		gpio_write(GPIO_B, 2, 1);
		gpio_write(GPIO_B, 4, 0);
	} else {
		gpio_write(GPIO_B, 4, 1);
		gpio_write(GPIO_B, 2, 0);
		speed *= -1;
	}
	pwm_set_duty(PWM1, speed);
}

void motor1_freq(uint32_t clk_speed, uint16_t freq) {
	pwm_set_freq(PWM0, clk_speed, freq);
}

void motor2_freq(uint32_t clk_speed, uint16_t freq) {
	pwm_set_freq(PWM1, clk_speed, freq);
}
