#ifndef NUNCHUCK_H
#define NUNCHUCK_H

#include "tiva_c.h"


struct nunchuck_state
{
	_Bool z;
	_Bool c;
	uint8_t x_joystick;
	uint8_t y_joystick;
	uint16_t x_tilt;
	uint16_t y_tilt;
	uint16_t z_tilt;
};

void nunchuck_init(uint32_t* i2c, uint32_t sys_clock);
struct nunchuck_state get_nunchuck_state(uint32_t* i2c, uint8_t address);
void nunchuck_read(uint32_t* i2c, uint8_t address);


#endif
