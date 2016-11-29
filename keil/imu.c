#include "imu.h"

uint8_t accel_address = 0x19;
uint8_t gryo_address = 0x6B;

void gyro_init(uint32_t* i2c, uint32_t clk_speed) {
	uint8_t data[2];
	i2c_init(i2c, clk_speed, I2C_400k);
	data[0] =  0x20;					// CTRL_REG1
	data[1] = 0x0F;						// 95 ODR 12.5 BW (same as Spark Fun)- PD off- z,x,y enable
	i2c_write(i2c, gryo_address, data, 2, 0);
	
	data[0] =  0x23;					// CTRL_REG4
	data[1] = 0x20;						// 2000 dps
	i2c_write(i2c, gryo_address, data, 2, 0);
}

int16_t get_x_angle(uint32_t* i2c){
	uint8_t s_address = 0xA8;			// x_angle address +8 for continuous read
	return get_gyro_2_byte_data(i2c, gryo_address, s_address);
}

int16_t get_y_angle(uint32_t* i2c){
	uint8_t s_address = 0xAA;			// y_angle address +8 for continuous read
	return get_gyro_2_byte_data(i2c, gryo_address, s_address);
}

int16_t get_z_angle(uint32_t* i2c){
	uint8_t s_address = 0xAC;			// z_angle address +8 for continuous read
	return get_gyro_2_byte_data(i2c, gryo_address, s_address);
}

uint16_t get_gyro_2_byte_data(uint32_t* i2c, uint8_t address, uint8_t s_address) {
	uint8_t data[2];
	i2c_read(i2c, address, s_address, data, 2);		// read two consecutive bytes of data
	return (int16_t) ((data[1] << 8) | (data[0]));	// create 16 bit data register
}

void accel_init(uint32_t* i2c, uint32_t clk_speed) {
	uint8_t data[2];
	i2c_init(i2c, clk_speed, I2C_400k);
	data[0] =  0x20;
	data[1] = 0x57;						// normal power mode, 400k Hz
	i2c_write(i2c, 0x19, data, 2, 0);
	
	data[0] =  0x23;
	data[1] = 0x08;						// high resolution (16 bit)
	i2c_write(i2c, 0x19, data, 2, 0);
	
}

int16_t get_x_accel(uint32_t* i2c) {
	uint8_t s_address = 0xA8;			// x_accell address +8 for continuous read
	return get_accel_2_byte_data(i2c, accel_address, s_address);
}

int16_t get_y_accel(uint32_t* i2c) {
	uint8_t s_address = 0xAA;			// x_accell address +8 for continuous read
	return get_accel_2_byte_data(i2c, accel_address, s_address);

}

int16_t get_z_accel(uint32_t* i2c) {
	uint8_t s_address = 0xAC;			// x_accell address +8 for continuous read
	return get_accel_2_byte_data(i2c, accel_address, s_address);

}

uint16_t get_accel_2_byte_data(uint32_t* i2c, uint8_t address, uint8_t s_address) {
	uint8_t data[2];
	i2c_read(i2c, address, s_address, data, 2);			// read two consecutive bytes of data
	return (int16_t) ((data[1] << 8) | (data[0])) >> 4;	// create 12 bit data register
}

