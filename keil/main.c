#include "tiva_c.h"

int main(void) {
	sys_clock(CLK_MOSC, CLK_PLL_ON, 5);
	i2c_init(I2C_0, 40000000, I2C_100k);
	
	//your code here.
	while(1);
}
