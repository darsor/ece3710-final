#include "tiva_c.h"
#include "motor.h"
#include "nunchuck.h"
#include "imu.h"
#include "pid.h"
#include <math.h>

#define RAD_TO_DEG 57.29578
#define GYRO_TO_DPS 0.00869750976 // (250.0 / 32768.0) * 1.14

uint32_t clk_speed = 40000000;
volatile float gr_angle = 0;
volatile float angle = 0;

int main(void) {	
    struct nunchuck_state state; 
    uint8_t which = 0; // increment p or i or d
    float delta = .000001;
    float PID[3] = {0,0,0};

	sys_clock(CLK_MOSC, CLK_PLL_ON, 5);

	uart_init(UART0, 115200, clk_speed);
	gyro_init(I2C_2, clk_speed);
	accel_init(I2C_2, clk_speed);
	timer_init(TIMER32_0, clk_speed/100, TIMER_PERIODIC);
    gpio_init(GPIO_F, 0x0E, GPIO_OUT, GPIO_DEN);
    nunchuck_init(I2C_1, clk_speed);

	//GPIO_F[0x38/4] = 0x2; //red - P
	//GPIO_F[0x38/4] = 0x4; //blue - D
	//GPIO_F[0x38/4] = 0x8; //green - I
	
	timer_timeout_int_en(TIMER32_0);
	nvic_int_en(19);
	timer_start(TIMER32_0);
	
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

void TIMER0A_Handler(void) {
    float xl_angle = 0;
    float dps;
    int16_t raw_gr_x;
    int16_t raw_xl_y;
    int16_t raw_xl_z;

    uint32_t timer_current = timer_value(TIMER32_0);
    timer_timeout_int_clr(TIMER32_0);

    raw_gr_x = get_x_angle(I2C_2) - 80;
    raw_xl_y = get_y_accel(I2C_2);
    raw_xl_z = get_z_accel(I2C_2);
    if (raw_xl_z == 0) raw_xl_z = 1;

    xl_angle = atan2(0-raw_xl_y, raw_xl_z) * RAD_TO_DEG;
    dps = raw_gr_x * GYRO_TO_DPS; // this is correct
    gr_angle -= dps * 0.01f;
    angle = 0.85f * (angle + dps*0.01f) + 0.15f * xl_angle;

    uprintf(UART0, "%4.4f,%4.4f,%4.4f\r\n", xl_angle, gr_angle, angle);
}
