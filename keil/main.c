#include "tiva_c.h"

uint32_t clk_speed = 40000000;

int main(void) {
	sys_clock(CLK_MOSC, CLK_PLL_ON, 5);
	
	//your code here.
	pwm_init(PWM0, clk_speed, 200);		// PB.6
	pwm_init(PWM1, clk_speed, 600);	// PB.4
	
	pwm_set_duty(PWM0, 0.73523);
	pwm_set_duty(PWM1, 0.365);
	
	pwm_set_freq(PWM0, clk_speed, 300);
	
	while(1);
}
