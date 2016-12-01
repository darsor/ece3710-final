#ifndef TESTS_H
#define TESTS_H

#include "tiva_c.h"

void test_motors(uint32_t* port, uint32_t clk_speed);
void test_nunchuck(uint32_t clk_speed);
void test_accel(uint32_t* i2c, uint32_t clk_speed);
void test_gyro(uint32_t* i2c, uint32_t clk_speed);

#endif
