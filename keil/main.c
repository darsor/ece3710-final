#include "tiva_c.h"

uint32_t clk_speed = 40000000;

int main(void) {
	sys_clock(CLK_MOSC, CLK_PLL_ON, 5);
	
	//your code here.
	pwm_init(PWM0, clk_speed, 500);		// PB.6
	pwm_init(PWM1, clk_speed, 20000);	// PB.4
	
	while(1);
}
