#include "nunchuck.h"

uint8_t joystick_x = 0x00;
uint8_t joystick_y = 0x01;
uint8_t tilt_x = 0x02;
uint8_t tilt_y = 0x03;
uint8_t tilt_z = 0x04;
uint8_t buttons = 0x05;

uint8_t addr = 0x52;

struct state enableNunchuck(uint32_t* i2c, uint32_t sys_clock, uint8_t speed)
{
	struct state forReturn;
	uint8_t* data;
	data[0] = 0x40;
	data[1] = 0x00;
	data[2] = 0x00;
	i2c_init(i2c, sys_clock, speed);
	forReturn = getState(i2c);
	return forReturn;
}

struct state getState(uint32_t* i2c)
{
		struct state forReturn;
	  uint8_t bools;
		forReturn.xJoystick = i2c_read(i2c, addr, joystick_x);
		forReturn.yJoystick = i2c_read(i2c, addr, joystick_y);
		forReturn.xTilt = i2c_read(i2c, addr, tilt_x);
		forReturn.yTilt = i2c_read(i2c, addr, tilt_y);
		bools = i2c_read(i2c, addr, buttons);
		forReturn.Z = bools & 0x01;
		forReturn.C = (bools >> 0x01) & 0x01;
		return forReturn;
}

uint8_t i2c_read(uint32_t* i2c, uint8_t address, uint8_t s_address) {
// TODO: High Speed mode
	i2c[0x000/4] = address << 1;
	i2c[0x008/4] = s_address;	// desired slave address
	i2c[0x004/4] = 0x03;			// start and transmit
	while (i2c_is_busy(i2c));
	
	i2c[0x004/4] = 0x23;			// start, receive, and stop
	while (i2c_is_busy(i2c));
	return i2c[0x008/4];			// return data register
}
