#ifndef IMU_H
#define IMU_H

#include "tiva_c.h"

void gyro_init(uint32_t* i2c, uint32_t clk_speed);
int16_t get_x_angle(uint32_t* i2c);
int16_t get_y_angle(uint32_t* i2c);
int16_t get_z_angle(uint32_t* i2c);
uint16_t get_gyro_2_byte_data(uint32_t* i2c, uint8_t address, uint8_t s_address);

void accel_init(uint32_t* i2c, uint32_t clk_speed);
int16_t get_x_accel(uint32_t* i2c);
int16_t get_y_accel(uint32_t* i2c);
int16_t get_z_accel(uint32_t* i2c);
uint16_t get_accel_2_byte_data(uint32_t* i2c, uint8_t address, uint8_t s_address);

#endif
