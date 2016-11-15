#include "tiva_c.h"

int main(void) {
	sys_clock(CLK_MOSC, CLK_PLL_ON, 5);
	
	//your code here.
	while(1);
}
