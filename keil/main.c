#include "tiva_c.h"
#include "motor.h"
#include "nunchuck.h"
#include "imu.h"
#include "pid.h"

uint32_t clk_speed = 40000000;

int main(void) {
	float speed = 0;
	uint16_t freq = 200;
	int16_t y_accel = 0;
	//uint16_t y_angle = 0;
	//int16_t freq_inc = 500;
	//float speed_inc = 0.1;
	//struct nunchuck_state state; 
	sys_clock(CLK_MOSC, CLK_PLL_ON, 5);
	gpio_init(GPIO_F, 0x11, GPIO_IN, GPIO_DEN | GPIO_PUR);
	//nunchuck_init(I2C_1, clk_speed);
	//gyro_init(I2C_2, clk_speed);
	accel_init(I2C_2, clk_speed);
	
	
	motors_init(clk_speed, freq);
	motor1_speed(speed);
	motor2_speed(speed);
	
	while(1) {
		y_accel = get_y_accel(I2C_2); // for motor 1
		//y_angle = get_y_angle(I2C_2); // for motor 2
		
		speed = y_accel / -1024.0;
		motor1_speed(speed);
		motor2_speed(speed);
		
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
