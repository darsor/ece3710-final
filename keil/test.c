#include "test.h"
#include "motor.h"
#include "nunchuck.h"
#include "imu.h"
#include "tiva_c.h"

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
	motors_init(clk_speed, 500);
	nunchuck_init(I2C_1, clk_speed);
	uart_init(UART4, 115200, clk_speed);
		
	while(1)
	{
		state = get_nunchuck_state(I2C_1, 0x052); // test nunchuck
		speed = (state.y_joystick/128.0 - 1);
		motor1_speed(speed);
		motor2_speed(speed);
		uprintf(UART4, "speed: %f\r\n", speed);
		msleep(25);
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

void test_imu_print(uint32_t clk_speed) {
	uart_init(UART0, 115200, clk_speed);
	gyro_init(I2C_2, clk_speed);
	accel_init(I2C_2, clk_speed);
	timer_init(TIMER32_4, clk_speed/200, TIMER_PERIODIC);
	timer_timeout_int_en(TIMER32_4);
	nvic_int_en(70);
	timer_start(TIMER32_4);
	
	while(1);
}

void TIMER4A_Handler(void) {
	int16_t raw_gr_x = get_x_angle(I2C_2)-10;
    int16_t raw_xl_y = get_y_accel(I2C_2);
    int16_t raw_xl_z = get_z_accel(I2C_2);
	uprintf(UART0, "gyro_x: %5d, accel_y: %5d, accel_z: %5d\r\n", raw_gr_x, raw_xl_y, raw_xl_z);
	timer_timeout_int_clr(TIMER32_4);
}

void test_motor_freq(uint32_t clk_speed) {
	uint32_t freq1 = 2000, freq2 = 2000;
	float speed = 0;
	struct nunchuck_state state;
	
	uart_init(UART4, 115200, clk_speed);
	nunchuck_init(I2C_1, clk_speed);
	motors_init(clk_speed, 2000);
	
	while(1) {
		state = get_nunchuck_state(I2C_1, 0x052);
		if(state.x_joystick < 0x20 && state.z) {
			freq1 += 100;
			if (freq1 > 2000) {
				freq1 = 100;
			}
		}
		else if(state.x_joystick > 0xD0 && state.z) {
			freq2 += 100;
			if (freq2 > 2000) {
				freq2 = 100;
			}
		}
		speed = (state.y_joystick/128.0 - 1);
		motor1_speed(speed);
		motor2_speed(speed);
		motor1_freq(clk_speed, freq1);
		motor2_freq(clk_speed, freq2);
		uprintf(UART4, "motor1: %5u Hz        motor2: %5u Hz\r\n", freq1, freq2);
		msleep(1250); // half second
	}
}
