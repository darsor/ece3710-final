#ifndef NUNCHUCK_H
#define NUNCHUCK_H

#include "tiva_c.h"


extern uint8_t joystick_x;
extern uint8_t joystick_y;
extern uint8_t tilt_x;
extern uint8_t tilt_y;
extern uint8_t tilt_z;
extern uint8_t buttons;

extern uint8_t addr;

struct state
{
	_Bool Z;
	_Bool C;
	unsigned char xJoystick;
	unsigned char yJoystick;
	unsigned char xTilt;
	unsigned char yTilt;
//	unsigned char zTilt;
};

struct state enableNunchuck(uint32_t* i2c, uint32_t sys_clock, uint8_t speed);
struct state getState(uint32_t* i2c);
uint8_t i2c_read(uint32_t* i2c, uint8_t address, uint8_t s_address);



#endif
