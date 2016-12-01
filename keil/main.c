#include "tiva_c.h"
#include "motor.h"
#include "nunchuck.h"
#include "imu.h"
#include "pid.h"
#include "test.h"

uint32_t clk_speed = 40000000;

int main(void) {
	struct nunchuck_state state; 
	uint8_t which = 0; // increment p or i or d
	float delta = .000001;
	float PID[3] = {0,0,0};
	sys_clock(CLK_MOSC, CLK_PLL_ON, 5);
//	test_nunchuck(clk_speed);
	gpio_init(GPIO_F, 0x0E, GPIO_OUT, GPIO_DEN);
	nunchuck_init(I2C_1, clk_speed);
	//gyro_init(I2C_2, clk_speed);
	//accel_init(I2C_2, clk_speed);
	//uart_init(UART6, 9600, clk_speed);
	
	//GPIO_F[0x38/4] = 0x2; //red - P
	//GPIO_F[0x38/4] = 0x4; //blue - D
	//GPIO_F[0x38/4] = 0x8; //green - I
	
	
	
	while(1) {
		msleep(1000);
		state = get_nunchuck_state(I2C_1,0x052);
		
		if(state.y_joystick > 0xD0)
		{
			//increase number
			PID[which] += delta;
		}
		else if(state.y_joystick < 0x20)
		{
			//decrease number
			if(PID[which]>0)PID[which] -= delta;
			if(PID[which]<0) PID[which] = 0;
		}
		
		if(state.x_joystick > 0xD0)
		{
			//increase increment number
			delta*=10;
		}
		else if(state.x_joystick < 0x20)
		{
			//decrease increment number
			delta/=10;
		}

		if(state.z)
		{
			//change number to change
			which+=1;
			if(which>2)which=0;
			if(which==0)GPIO_F[0x38/4] = 0x2; //if P, then red
			if(which==1)GPIO_F[0x38/4] = 0x8; //if I, then green
			if(which==2)GPIO_F[0x38/4] = 0x4; //if D, then BLUE
		}

		//send to uart
		//uprintf(UART6, "P = %c I = %c D = %c delta = ", PID[0], PID[1], PID[2], delta);
	}
}
