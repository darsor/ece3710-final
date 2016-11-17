#include "nunchuck.h"

int main(void) {
	struct state current_state;
	
	sys_clock(CLK_MOSC, CLK_PLL_ON, 5);
	
	current_state = enableNunchuck(I2C_1, 40000000, I2C_100k);

	while(1)
	{
		msleep(2500);
		current_state = getState(I2C_1);
	}
	
}
