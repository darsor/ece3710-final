#include "tiva_c.h"
#include "motor.h"
#include "nunchuck.h"
#include "imu.h"
#include "pid.h"
#include "test.h"
#include <math.h>

#define RAD_TO_DEG 57.29578
#define GYRO_TO_DPS 0.00869750976 // (250.0 / 32768.0) * 1.14

uint32_t clk_speed = 40000000;
//volatile float gr_angle = 0;
volatile float angle = 0;
volatile float PID[3] = {0.01,0,0};
volatile float delta = .000001;
volatile int inc = 0;
volatile uint8_t which = 0; // increment p or i or d

int main(void) {	
	sys_clock(CLK_MOSC, CLK_PLL_ON, 5);

	uart_init(UART4, 115200, clk_speed);
	gyro_init(I2C_2, clk_speed);
	accel_init(I2C_2, clk_speed);
	timer_init(TIMER32_0, clk_speed/500, TIMER_PERIODIC);
	timer_init(TIMER32_1, clk_speed/2.5, TIMER_PERIODIC);
    gpio_init(GPIO_F, 0x0E, GPIO_OUT, GPIO_DEN);
    nunchuck_init(I2C_1, clk_speed);
	motors_init(clk_speed, 20000);
	
	initialize_pid(PID[0], PID[1], PID[2], 0.002);
	set_limits(-1.0, 1.0);
	set_deadzone(-0.54, 0.54);
	
	timer_timeout_int_en(TIMER32_0);
	timer_timeout_int_en(TIMER32_1);
	nvic_int_en(19);
	nvic_int_en(21);
	nvic_set_pri(21, 7);
	timer_start(TIMER32_0);
	timer_start(TIMER32_1);
	
	gpio_write(GPIO_F, 1, 1);
	
	while(1);
}

void TIMER0A_Handler(void) {
    float xl_angle = 0;
    float dps, speed;
    int16_t raw_gr_x;
    int16_t raw_xl_y;
    int16_t raw_xl_z;

    //uint32_t timer_current = timer_value(TIMER32_0);
    timer_timeout_int_clr(TIMER32_0);

    raw_gr_x = get_x_angle(I2C_2) - 80;
    raw_xl_y = get_y_accel(I2C_2);
    raw_xl_z = get_z_accel(I2C_2);
    if (raw_xl_z == 0) raw_xl_z = 1;

    xl_angle = atan2(raw_xl_y, raw_xl_z) * RAD_TO_DEG;
    dps = raw_gr_x * GYRO_TO_DPS; // this is correct
    //gr_angle = dps * 0.005f;
    angle = 0.85f * (angle + dps*0.002f) + 0.15f * xl_angle;
	
	speed = pid_update(0.0, angle);
	motor1_speed(speed);
	motor2_speed(speed);
	/*if (++inc > 200) { // verify that PID loop is fast enough
		inc = 0;
		uprintf(UART4, "\ntimer: %u\n", timer_current);
	} */
	//uprintf(UART4, "    Kp = %09.6f, Ki = %09.6f Kd = %09.6f delta = %09.6f\r\n", PID[0], PID[1], PID[2], delta);
}

void TIMER1A_Handler(void) {
	struct nunchuck_state state; 
	timer_timeout_int_clr(TIMER32_1);
	
	state = get_nunchuck_state(I2C_1,0x052);
	
	if(state.y_joystick > 0xD0) {
		//increase number
		PID[which] += delta;
	}
	else if(state.y_joystick < 0x20) {
		//decrease number
		if(PID[which]>0) PID[which] -= delta;
		if(PID[which]<0) PID[which] = 0;
	}
	
	if(state.x_joystick < 0x20) {
		//increase increment number
		delta*=10;
	}
	else if(state.x_joystick > 0xD0) {
		//decrease increment number
		if (delta > 0.000001f) delta/=10;
	}

	if(state.z) {
		//change controller constant
		which = (which+1)%3;
		if(which==0) {
			gpio_write(GPIO_F, 2, 0);
			gpio_write(GPIO_F, 1, 1); //if P, then red
		} else if(which==1) {
			gpio_write(GPIO_F, 1, 0);
			gpio_write(GPIO_F, 3, 1); //if I, then green
		} else if(which==2) {
			gpio_write(GPIO_F, 3, 0);
			gpio_write(GPIO_F, 2, 1); //if D, then blue
		}
	}
	
	if (state.c) {
		reset_i_term();
	}

	initialize_pid(PID[0], PID[1], PID[2], 0.01);
	uprintf(UART4, "                                                                                      Kp = %09.6f, Ki = %09.6f Kd = %09.6f delta = %09.6f\r\n", PID[0], PID[1], PID[2], delta);
}
