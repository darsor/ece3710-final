#include "tiva_c.h"
#include "motor.h"

uint32_t clk_speed = 40000000;

int main(void) {
	float speed = 0;
	uint16_t freq = 200;
	int16_t freq_inc = 500;
	float speed_inc = 0.1;
	sys_clock(CLK_MOSC, CLK_PLL_ON, 5);
	gpio_init(GPIO_F, 0x11, GPIO_IN, GPIO_DEN | GPIO_PUR);
	
	//your code here.
	
	motors_init(clk_speed, freq);
	motor1_speed(speed);
	
	while(1) {
		if (!gpio_read(GPIO_F, 4)) {
			speed += speed_inc;
			if (speed >= 1) {
				speed = 0.99;
				speed_inc = -0.1;
			} else if (speed <= -1) {
				speed = -0.99;
				speed_inc = 0.1;
			}
			motor1_speed(speed);
			msleep(1000);
		}
		if (!gpio_read(GPIO_F, 0)) {
			freq += freq_inc;
			if (freq >= 4000) {
				freq = 4000;
				freq_inc = -500;
			} else if (freq <= 500) {
				freq = 500;
				freq_inc = 500;
			}
			motor1_freq(clk_speed, freq);
			msleep(1000);
		}
	}
}
