#include "tiva_c.h"
#include "motor.h"
#include "nunchuck.h"
#include "imu.h"

uint32_t clk_speed = 40000000;

int main(void) {
	//float speed = 0;
	//float speed2 = 0;	
	//uint16_t freq = 200;
	//int16_t y_accel = 0;
	//int16_t x_angle = 0;
	//int16_t y_angle = 0;
	//int16_t freq_inc = 500;
	//float speed_inc = 0.1;
	struct nunchuck_state state; 
	uint8_t which = 0; // increment p or i or d
	float delta = .000001;
	float PID[3] = {0,0,0};
	sys_clock(CLK_MOSC, CLK_PLL_ON, 5);
	//gpio_init(GPIO_F, 0x11, GPIO_IN, GPIO_DEN | GPIO_PUR);
	gpio_init(GPIO_F, 0x1F, GPIO_OUT, GPIO_DEN);
	//gpio_odr(GPIO_F, 0x0E, 1);
	//nunchuck_init(I2C_1, clk_speed);
	gyro_init(I2C_2, clk_speed);
	//accel_init(I2C_2, clk_speed);
	uart_init(UART6, 9600, clk_speed);
	
	//GPIO_F[0x38/4] = 0x2; //red - P
	//GPIO_F[0x38/4] = 0x4; //blue - D
	//GPIO_F[0x38/4] = 0x8; //green - I
	
	//motors_init(clk_speed, freq);
	//motor1_speed(speed);
	//motor2_speed(speed);

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
		uprintf(UART6, "P = %c I = %c D = %c delta = ", PID[0], PID[1], PID[2], delta);
		
		/*x_angle = get_x_angle(I2C_2); // for motor 2 //test gyro
		//y_angle = get_y_angle(I2C_2); // for motor 2
		
		speed = x_angle / -2048.0;
		//speed2 = y_angle / -1024.0;
		motor1_speed(speed); 
		motor2_speed(speed);*/
		
		/*y_accel = get_y_accel(I2C_2); // for motor 1 // test accel
		//y_angle = get_y_angle(I2C_2); // for motor 2
		
		speed = y_accel / -1024.0;
		motor1_speed(speed);
		motor2_speed(speed);*/
		
		/*state = get_nunchuck_state(I2C_1, 0x052); // test nunchuck
		speed = state.y_joystick/128.0 - 1;
		motor1_speed(speed);
		motor2_speed(speed);*/
		//msleep(1000);
		/*if (!gpio_read(GPIO_F, 4)) { // test motors
			speed += speed_inc;
			if (speed >= 1) {
				speed = 0.99;
				speed_inc = -0.1;
			} else if (speed <= -1) {
				speed = -0.99;
				speed_inc = 0.1;
			}
			motor1_speed(speed);
			motor2_speed(speed);
			msleep(1000);
		}
		if (!gpio_read(GPIO_F, 0)) {
			freq += freq_inc;
			if (freq >= 4000) {
				freq = 4000;
				freq_inc = -500;
			} else if (freq <= 500) {
				freq = 500;
				freq_inc = 500;
			}
			motor1_freq(clk_speed, freq);
			motor2_freq(clk_speed, freq);
			msleep(1000);
		}*/
	}
}
