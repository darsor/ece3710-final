#include "test.h"
#include "motor.h"
#include "nunchuck.h"
#include "imu.h"

void test_motors(uint32_t* port, uint32_t clk_speed)
{
	float speed = 0;
	float speed_inc = 0.1;
	uint16_t freq = 200;	
	int16_t freq_inc = 500;	
	
	gpio_init(GPIO_F, 0x11, GPIO_IN, GPIO_DEN | GPIO_PUR);	
	motors_init(clk_speed, freq);
	motor1_speed(speed);
	motor2_speed(speed);
	while(1)
	{
		if (!gpio_read(GPIO_F, 4)) { // test motors
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
		}
	}
}

void test_nunchuck(uint32_t clk_speed)
{
	float speed = 0;
	struct nunchuck_state state; 
	nunchuck_init(I2C_1, clk_speed);
		
	while(1)
	{
		state = get_nunchuck_state(I2C_1, 0x052); // test nunchuck
		speed = state.y_joystick/128.0 - 1;
		motor1_speed(speed);
		motor2_speed(speed);	
	}
}		

void test_accel(uint32_t* i2c, uint32_t clk_speed)
{
		float speed = 0;
		int16_t y_accel = 0;
		accel_init(i2c, clk_speed);
	
	while(1)
	{
		y_accel = get_y_accel(i2c); // for motor 1 // test accel
		
		speed = y_accel / -1024.0;
		motor1_speed(speed);
		motor2_speed(speed);
	}
}

void test_gyro(uint32_t* i2c, uint32_t clk_speed)
{
	int16_t x_angle = 0;
	int16_t y_angle = 0;
	float speed = 0;
	float speed2 = 0;	
	
	gyro_init(i2c, clk_speed);
	
	while(1)
	{
		x_angle = get_x_angle(i2c); // for motor 2 
		y_angle = get_y_angle(i2c); // for motor 2
	
	
		speed = x_angle / -2048.0;
		speed2 = y_angle / -1024.0;
		motor1_speed(speed); 
		motor2_speed(speed2);
	}
}
