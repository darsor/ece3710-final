#include "nunchuck.h"

uint8_t address = 0x52;

void nunchuck_init(uint32_t* i2c, uint32_t clk_speed) {
	uint8_t data[2];
	i2c_init(i2c, clk_speed, I2C_100k);
	msleep(1);

	data[0] = 0xF0;
	data[1] = 0x55;
	i2c_write(i2c, 0x52, data, 2, 0);
	
	data[0] = 0xFB;
	data[1] = 0x55;
	i2c_write(i2c, 0x52, data, 2, 0);
}

struct nunchuck_state get_nunchuck_state(uint32_t* i2c, uint8_t address) {
		struct nunchuck_state forReturn;
		uint8_t nunchuck_data[6];
		uint8_t data;
	
		data = 0x00;								// send command to remote to take new sample
		i2c_write(i2c, 0x52, &data, 1, 0);
		msleep(12);									// wait for new sample
		
		i2c_read(i2c, address, 0x00, nunchuck_data, 6);
		forReturn.x_joystick = nunchuck_data[0];
		forReturn.y_joystick = nunchuck_data[1];
		forReturn.x_tilt = (nunchuck_data[2] << 2 | (nunchuck_data[5] >> 6));
		forReturn.y_tilt = (nunchuck_data[3] << 2 | ((nunchuck_data[5] >> 4) & 0x03));
		forReturn.z_tilt = (nunchuck_data[4] << 2 | ((nunchuck_data[5] >> 2) & 0x03));
		forReturn.z = !(nunchuck_data[5] & 0x01);
		forReturn.c = !((nunchuck_data[5] >> 0x01) & 0x01);
		return forReturn;
}
