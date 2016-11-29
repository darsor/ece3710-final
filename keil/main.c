#include "tiva_c.h"
#include "motor.h"
#include "nunchuck.h"
#include "pid.h"

uint32_t clk_speed = 40000000;

int main(void) {
    sys_clock(CLK_MOSC, CLK_PLL_ON, 5);

	while(1);
}
