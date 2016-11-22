#include "nunchuck.h"


uint32_t clk_speed = 40000000;


int main(void) {
	struct nunchuck_state current_state;
	sys_clock(CLK_MOSC, CLK_PLL_ON, 5);
	nunchuck_init(I2C_1, clk_speed);
	while(1) {
		current_state = get_nunchuck_state(I2C_1, 0x52);
	}
	
}
