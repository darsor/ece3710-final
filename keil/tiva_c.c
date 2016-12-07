#include "tiva_c.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

uint32_t* SYS_CTL = (uint32_t*) 0x400FE000;
uint32_t* CORE_P  = (uint32_t*) 0xE000E000;

uint8_t CLK_MOSC	= 0x00;
uint8_t CLK_PIOSC	= 0x01;
uint8_t CLK_PLL_ON	= 0x01;
uint8_t CLK_PLL_OFF	= 0x00;

uint32_t* GPIO_A = (uint32_t*) 0x40004000;
uint32_t* GPIO_B = (uint32_t*) 0x40005000;
uint32_t* GPIO_C = (uint32_t*) 0x40006000;
uint32_t* GPIO_D = (uint32_t*) 0x40007000;
uint32_t* GPIO_E = (uint32_t*) 0x40024000;
uint32_t* GPIO_F = (uint32_t*) 0x40025000;

uint8_t GPIO_OUT = 0xFF;
uint8_t GPIO_IN  = 0x00;
uint8_t GPIO_TRI = 0x00;
uint8_t GPIO_PUR = 0x01;
uint8_t GPIO_PDR = 0x02;
uint8_t GPIO_ODR = 0x04;
uint8_t GPIO_DEN = 0x10;
uint8_t GPIO_NONE= 0x20;

uint8_t GPIO_RISING = 0xFF;
uint8_t GPIO_FALLING = 0x00;
uint8_t GPIO_BOTH = 0x01;

uint32_t* UART0 = (uint32_t*) 0x4000C000;
uint32_t* UART1 = (uint32_t*) 0x4000D000;
uint32_t* UART2 = (uint32_t*) 0x4000E000;
uint32_t* UART3 = (uint32_t*) 0x4000F000;
uint32_t* UART4 = (uint32_t*) 0x40010000;
uint32_t* UART5 = (uint32_t*) 0x40011000;
uint32_t* UART6 = (uint32_t*) 0x40012000;
uint32_t* UART7 = (uint32_t*) 0x40013000;

uint32_t* I2C_0 = (uint32_t*) 0x40020000;
uint32_t* I2C_1 = (uint32_t*) 0x40021000;
uint32_t* I2C_2 = (uint32_t*) 0x40022000;
uint32_t* I2C_3 = (uint32_t*) 0x40023000;

uint32_t I2C_100k      = 100000;
uint32_t I2C_400k      = 400000;
uint32_t I2C_1000k 	   = 1000000;
uint32_t I2C_HIGHSPEED = 3330000;

uint32_t* TIMER32_0 = (uint32_t*) 0x40030000;
uint32_t* TIMER32_1 = (uint32_t*) 0x40031000;
uint32_t* TIMER32_2 = (uint32_t*) 0x40032000;
uint32_t* TIMER32_3 = (uint32_t*) 0x40033000;
uint32_t* TIMER32_4 = (uint32_t*) 0x40034000;
uint32_t* TIMER32_5 = (uint32_t*) 0x40035000;

uint8_t TIMER_ONESHOT  = 0x01;
uint8_t TIMER_PERIODIC = 0x02;

uint32_t* ADC0 = (uint32_t*) 0x40038000;
uint32_t* ADC1 = (uint32_t*) 0x40039000;

uint32_t* SPI0 = (uint32_t*) 0x40008000;
uint32_t* SPI1 = (uint32_t*) 0x40009000;
uint32_t* SPI2 = (uint32_t*) 0x4000A000;
uint32_t* SPI3 = (uint32_t*) 0x4000B000;

uint8_t SPI_ACTIVE_LOW	= 0x40;
uint8_t SPI_ACTIVE_HIGH	= 0x00;
uint8_t SPI_CLK_FIRST	= 0x00;
uint8_t SPI_CLK_SECOND	= 0x80;

uint32_t* PWM_BASE = (uint32_t*) 0x40028000;
uint32_t* PWM0	   = (uint32_t*) 0x40028040;
uint32_t* PWM1 	   = (uint32_t*) 0x40028080;
uint32_t* PWM2 	   = (uint32_t*) 0x400280C0;
uint32_t* PWM3 	   = (uint32_t*) 0x40028100;

void sys_clock(uint8_t src, uint8_t pll, uint8_t sysdiv) {
	uint32_t rcc = SYS_CTL[0x060/4];
	rcc &= 0xF83FF80E;					// clear SYSDIV, USESYSDIV, XTAL, OSCSRC, and MOSCDIS bits
	rcc |= 0x540;						// set XTAL to 16MHz
	rcc |= src << 4;					// set OSCSRC
	rcc |= src;							// set MOSCDIS
	if (sysdiv != 0x00) {
		rcc |= (sysdiv - 1) << 23;		// set SYSDIV
		rcc |= 1 << 22;					// set USESYSDIV
	}
	SYS_CTL[0x060/4] = rcc;
	
	if (pll == CLK_PLL_ON) {
		SYS_CTL[0x060/4] &= 0xFFFFD7FF;	// turn on PLL and switch to it
	}
}

void nvic_int_en(uint8_t interrupt) {
	uint16_t offset = (interrupt - (interrupt % 32)) >> 3;
	offset += 0x100;
	CORE_P[offset/4] |= 1 << (interrupt % 32);
}

void nvic_set_pri(uint8_t interrupt, uint8_t priority) {
	uint16_t offset = interrupt - (interrupt % 4);
	offset += 0x400;
	CORE_P[offset/4] |= (priority << 5) << ((interrupt % 4) << 3);
}

void systick_init(uint32_t reload) {
	CORE_P[0x10/4] &= ~((uint32_t) 1);	// disable systick
	CORE_P[0x14/4] = reload;			// set reload value
	CORE_P[0x10/4] |= 1;				// enable systick
}

uint32_t systick_current() {
	return SYS_CTL[0x18/4];
}

void gpio_init(uint32_t* port, uint8_t pins, uint8_t dir, uint8_t state) {
	gpio_clock(port);
	gpio_unlock(port, pins);
	
	port[0x400/4] |= (pins & dir);				// set direction
	
	port[0x420/4] &= ~((uint32_t) pins);		// disable alternate functions
	
	if (state != GPIO_NONE) {
		if (state & GPIO_PUR) gpio_pur(port, pins, 1);	// set PUR
		else gpio_pur(port, pins, 0);					// clear PUR
			
		if (state & GPIO_PDR) gpio_pdr(port, pins, 1);	// set PDR
		else gpio_pdr(port, pins, 0);					// clear PDR
			
		if (state & GPIO_ODR) gpio_odr(port, pins, 1);	// set ODR
		else gpio_odr(port, pins, 0);					// clear ODR
			
		if (state & GPIO_DEN) gpio_den(port, pins, 1);	// set DEN
		else gpio_den(port, pins, 0);					// clear DEN
	}
}

void gpio_int_edge_en(uint32_t* port, uint8_t pins, uint8_t edge) {
	if (edge == GPIO_BOTH) {
		port[0x408/4] |= pins;			// enable trigger on both edges
	} else {
		port[0x40C/4] |= (pins & edge); // set rising or falling edge
	}
	port[0x410/4] |= pins;
}

void gpio_int_edge_clr(uint32_t* port, uint8_t pins) {
	port[0x41C/4] |= pins;
}

void gpio_clock(uint32_t* port) {
	uint8_t mask;
	if (port == GPIO_A) mask = 0x01;
	else if (port == GPIO_B) mask = 0x02;
	else if (port == GPIO_C) mask = 0x04;
	else if (port == GPIO_D) mask = 0x08;
	else if (port == GPIO_E) mask = 0x10;
	else if (port == GPIO_F) mask = 0x20;
	SYS_CTL[0x608/4] |= mask;
}

void gpio_unlock(uint32_t* port, uint8_t pins) {
	port[0x520/4] = 0x4C4F434B;					// unlock port GPIOCR register
	port[0x524/4] |= pins;						// unlock pins on that port
}

void gpio_pur(uint32_t* port, uint8_t pins, uint8_t state) {
	if (state) port[0x510/4] |= pins;				// set PUR
	else port[0x510/4] &= ~((uint32_t) pins);		// clear PUR
}

void gpio_pdr(uint32_t* port, uint8_t pins, uint8_t state) {
	if (state) port[0x514/4] |= pins;				// set PDR
	else port[0x514/4] &= ~((uint32_t) pins);		// clear PDR
}

void gpio_odr(uint32_t* port, uint8_t pins, uint8_t state) {
	if (state) port[0x50C/4] |= pins;				// set ODR
	else port[0x50C/4] &= ~((uint32_t) pins);		// clear ODR
}

void gpio_den(uint32_t* port, uint8_t pins, uint8_t state) {
	if (state) port[0x51C/4] |= pins;				// set DEN
	else port[0x51C/4] &= ~((uint32_t) pins);		// clear DEN
}

void gpio_write(uint32_t* port, uint8_t pin, uint8_t value) {
	port[(1 << pin << 2)/4] = value << pin;
}

void gpio_afsel(uint32_t* port, uint8_t pins, uint8_t state) {
	if (state) {
		port[0x420/4] |= pins;
	} else {
		port[0x420/4] &= ~pins;
	}
}

uint8_t gpio_read(uint32_t* port, uint8_t pin) {
	return (port[0x3FC/4] >> pin) & 1;
}

// assumes FIFO and 8N1 are desired
void uart_init(uint32_t* uart, uint32_t baud, uint32_t sys_clock) {
	uint8_t clk_mask;
	uint32_t* gpio_port;
	uint8_t gpio_mask;
	uint32_t alt_func;
	uint16_t brdi;
	float brd;
	uint8_t brdf;
	if (uart == UART0) {
		clk_mask = 0x01;
		gpio_port = GPIO_A;
		gpio_mask = 0x03;
	} else if (uart == UART1) {
		clk_mask = 0x02;
		gpio_port = GPIO_B;
		gpio_mask = 0x03;
	} else if (uart == UART2) {
		clk_mask = 0x04;
		gpio_port = GPIO_D;
		gpio_mask = 0xC0;
	} else if (uart == UART3) {
		clk_mask = 0x08;
		gpio_port = GPIO_C;
		gpio_mask = 0xC0;
	} else if (uart == UART4) {
		clk_mask = 0x10;
		gpio_port = GPIO_C;
		gpio_mask = 0x30;
	} else if (uart == UART5) {
		clk_mask = 0x20;
		gpio_port = GPIO_E;
		gpio_mask = 0x30;
	} else if (uart == UART6) {
		clk_mask = 0x30;
		gpio_port = GPIO_D;
		gpio_mask = 0x30;
	} else if (uart == UART7) {
		clk_mask = 0x40;
		gpio_port = GPIO_E;
		gpio_mask = 0x03;
	}
	SYS_CTL[0x618/4] |= clk_mask;		// route clock to specified UART

	gpio_clock(gpio_port);				// start clock on GPIO port
	gpio_unlock(gpio_port, gpio_mask);	// make sure pins are unlocked
		
	gpio_port[0x420/4] |= gpio_mask;	// enable alt functions on UART pins
	
	alt_func = gpio_port[0x52C/4];		// read-modify-write alt. functions
	if (gpio_mask == 0x03) {
		alt_func &= 0xFFFFFF00;
		alt_func |= 0x00000011;
	} else if (gpio_mask == 0x30) {
		alt_func &= 0xFF00FFFF;
		alt_func |= 0x00110000;
	} else if (gpio_mask == 0xC0) {
		alt_func &= 0x00FFFFFF;
		alt_func |= 0x11000000;
	}
	gpio_port[0x52C/4] = alt_func;
	
	gpio_den(gpio_port, gpio_mask, 1);
	
	uart[0x30/4] &= ~((uint32_t) 0x1);	// disable UART
	
	brd = ((float)sys_clock) / (baud << 4);		// calculate baud divisor
	brdi = brd;
	brdf = 64 * (brd - brdi) + 0.5f;
	
	uart[0x24/4] = brdi;				// set baud rate
	uart[0x28/4] = brdf;
	uart[0x2C/4] = 0x70;				// 8-bit frame and FIFO enable (8N1)
	uart[0x30/4] |= 5;					// enable UART (with system clock)
}

// send byte over UART
void uart_send(uint32_t* uart, uint8_t data) {
	while (uart_tx_is_full(uart));			// wait if FIFO is full
	uart[0] = data;
}

// receive byte from UART (blocks until received)
void uart_receive(uint32_t* uart, uint8_t* data) {
	while (uart_rx_is_empty(uart));			// wait if FIFO is empty
	*data = uart[0];
}

uint8_t uart_rx_is_empty(uint32_t* uart) {
	return uart[0x18/4] & 0x10;
}

uint8_t uart_tx_is_full(uint32_t* uart) {
	return uart[0x18/4] & 0x20;
}

// send null-terminated string over specified UART
void uart_send_stream(uint32_t* uart, unsigned char* str) {
	uint32_t i = 0;
	while (str[i]) {
		uart_send(uart, str[i++]);
	}
}

void uart_rx_int(uint32_t* uart) {
	uart[0x038/4] |= 0x10;			// mask UART RX interrupt
	uart[0x034/4] &= 0x0E;			// set trigger at RX FIFO >= 1/8
}

int uprintf(uint32_t* uart, const char* format, ...) {
	int ret;
	char string[256];
	va_list args;
	va_start(args, format);
	ret = vsnprintf(string, 256, format, args);
	va_end(args);
	uart_send_stream(uart, (unsigned char*) string);
	return ret;
}

void timer_init(uint32_t* timer, uint32_t reload, uint8_t mode) {
	uint8_t clock_mask;
	if (timer == TIMER32_0) {
		clock_mask = 0x01;
	} else if (timer == TIMER32_1) {
		clock_mask = 0x02;
	} else if (timer == TIMER32_2) {
		clock_mask = 0x04;
	} else if (timer == TIMER32_3) {
		clock_mask = 0x08;
	} else if (timer == TIMER32_4) {
		clock_mask = 0x10;
	} else if (timer == TIMER32_5) {
		clock_mask = 0x20;
	}
	SYS_CTL[0x604/4] |= clock_mask;		// route clock to timer module
	
	timer_stop(timer);					// disable timer
	timer[0x000/4] = 0x00;				// select 32-bit mode
	timer[0x004/4] &= ~((uint32_t) 3);	// clear the TAMR bits
	timer[0x004/4] |= mode;				// set periodic or one-shot
	// the timer counts down by default
	timer[0x028/4] = reload;			// set reload value
}

uint32_t timer_value(uint32_t* timer) {
	return timer[0x050/4];
}

void timer_start(uint32_t* timer) {
	timer[0x00C/4] |= 1;				// enable timer
}

void timer_stop(uint32_t* timer) {
	timer[0x00C/4] &= ~((uint32_t) 1);	// disable timer
}

void timer_timeout_int_en(uint32_t* timer) {
	timer[0x018/4] |= 1;				// enable timer A timeout interrupt
}

void timer_timeout_int_clr(uint32_t* timer) {
	timer[0x024/4] |= 1;				// clear timer A timeout interrupt
}

uint8_t timer_expired(uint32_t* timer) {
	return timer[0x01C/4] & 1;
}

void i2c_init(uint32_t* i2c, uint32_t sys_clock, uint32_t speed) {
	uint8_t clock_mask;
	uint32_t* gpio_port;
	uint8_t gpio_pins;
	uint32_t alt_func;
	if (i2c == I2C_0) {
		clock_mask = 0x01;
		gpio_port = GPIO_B;
		gpio_pins = 0x0C;
	} else if (i2c == I2C_1) {
		clock_mask = 0x02;
		gpio_port = GPIO_A;
		gpio_pins = 0xC0;
	} else if (i2c == I2C_2) {
		clock_mask = 0x04;
		gpio_port = GPIO_E;
		gpio_pins = 0x30;
	} else if (i2c == I2C_3) {
		clock_mask = 0x08;
		gpio_port = GPIO_D;
		gpio_pins = 0x03;
	}
	SYS_CTL[0x620/4] |= clock_mask;
	
	gpio_clock(gpio_port);
	gpio_port[0x420/4] |= gpio_pins;	// enable alternate functions on pins
	gpio_odr(gpio_port, gpio_pins & ~(gpio_pins >> 1), 1);	// enable open drain on SDA
	gpio_pur(gpio_port, gpio_pins, 1); // TODO: is this necessary?
	gpio_den(gpio_port, gpio_pins, 1);
	
	alt_func = gpio_port[0x52C/4];		// read-modify-write alt. functions
	if (gpio_pins == 0x03) {
		alt_func &= 0xFFFFFF00;
		alt_func |= 0x00000033;
	} else if (gpio_pins == 0x30) {
		alt_func &= 0xFF00FFFF;
		alt_func |= 0x00330000;
	} else if (gpio_pins == 0xC0) {
		alt_func &= 0x00FFFFFF;
		alt_func |= 0x33000000;
	} else if (gpio_pins == 0x0C) {
		alt_func &= 0xFFFF00FF;
		alt_func |= 0x00003300;
	}
	gpio_port[0x52C/4] = alt_func;
	
	i2c[0x020/4] = 0x10;				// enable master function
	i2c[0x00C/4] = 0x13;// | (speed == I2C_HIGHSPEED) ? 0x80 : 0x00; // set speed
}

uint8_t i2c_is_busy(uint32_t* i2c) {
	return i2c[0x004/4] & 0x1;
}

void i2c_read(uint32_t* i2c, uint8_t address, uint8_t s_address, uint8_t* data, uint32_t size) {
	uint8_t i=0;
			
	i2c[0x000/4] = address << 1;
	i2c[0x008/4] = s_address;
	i2c[0x004/4] = 0x03;			// start, transmit
	while (i2c_is_busy(i2c));
	
	if (size == 1) {
        i2c[0x000/4] = i2c[0x000/4] | 0x01;	
        i2c[0x004/4] = 0x03;	    // re-start, receive, negative ACK
        while (i2c_is_busy(i2c));
        data[0] = i2c[0x008/4];
        i2c[0x004/4] = 0x04;		// stop
        return;
	}
	
	i2c[0x000/4] = i2c[0x000/4] | 0x01;
	i2c[0x004/4] = 0x0B;			// re-start, receive,
	while (i2c_is_busy(i2c));
	data[0] = i2c[0x008/4];
	
	for (i=1; i<size-1; ++i) {
		i2c[0x004/4] = 0x09;		// receive next byte
		while (i2c_is_busy(i2c));
		data[i] = i2c[0x008/4];
	}
	
	i2c[0x004/4] = 0x01;			// receive and no ACK
	while (i2c_is_busy(i2c));
	data[size-1] = i2c[0x008/4];
	
	i2c[0x004/4] = 0x04;			// stop
	while (i2c_is_busy(i2c));
}

void i2c_write(uint32_t* i2c, uint8_t address, uint8_t* data, uint8_t size, uint8_t HS) {
	uint8_t i=0;
	if (HS) {
		i2c[0x000/4] = 0x08;
		i2c[0x004/4] = 0x13;
		while (i2c_is_busy(i2c));
	}

	i2c[0x000/4] = address << 1;
	i2c[0x008/4] = data[0];
	if (size == 1) {
		i2c[0x004/4] = 0x07;
		while (i2c_is_busy(i2c));
		return;
	}
	i2c[0x004/4] = 0x03;			// transmit start byte and address, and first data byte
	while (i2c_is_busy(i2c));
	for (i=1; i<size-1; ++i) {
		i2c[0x008/4] = data[i];
		i2c[0x004/4] = 0x01;		// transmit next byte
		while (i2c_is_busy(i2c));
	}
	i2c[0x008/4] = data[i];
	i2c[0x004/4] = 0x05;			// transmit last byte
	while (i2c_is_busy(i2c));
}

uint32_t* bitband(uint32_t* address, uint8_t bit) {
	uint32_t bitband_addr = (uint32_t) address;
	if (bitband_addr > 0x40000000) {
		bitband_addr -= 0x40000000;
		bitband_addr <<= 5;
		bitband_addr += 0x42000000;
	} else {
		bitband_addr -= 0x20000000;
		bitband_addr <<= 5;
		bitband_addr += 0x22000000;
	}
	bitband_addr += (bit << 2);
	return (uint32_t*) bitband_addr;
}

// set up PE.0 ADC input, sampling at timeout of specified timer
void adc_init(uint32_t* adc, uint32_t* timer) {
	uint8_t clock_mask;
	if (adc == ADC0) {
		clock_mask = 0x01;
	} else if (adc == ADC1) {
		clock_mask = 0x02;
	}
	SYS_CTL[0x638/4] |= clock_mask;
	
	gpio_clock(GPIO_E);
	gpio_unlock(GPIO_E, 0x01);
	GPIO_E[0x420/4] |= 0x01;	// enable alternate functions on PE.0
	gpio_den(GPIO_E, 0x01, 0);
	GPIO_E[0x528/4] |= 0x01;	// disable analog isolation circuit
	
	adc[0x000/4] = 0x00;		// disable sequencers for configuration
	adc[0x014/4] = 0x05;		// enable timer trigger for sequencer 0
	
	timer[0x00C/4] |= 0x20;		// set timer to trigger ADC
	
	adc[0x040/4] = 0x03;		// set SEQ0 sample 1 to AIN3 (PE.0)
	adc[0x044/4] = 0x06;		// set SEQ0 sample 1 as last sample with interrupts
	adc[0x008/4] |= 0x01;		// enable SEQ0 interrupt
	nvic_int_en(14);			// enable SEQ0 interrupt in NVIC
	adc[0x000/4] = 0x01;		// enable SEQ0
}

// return ADC result from SEQ0
uint16_t adc_read(uint32_t* adc) {
	return adc[0x048/4];
}

// clear SEQ0 interrupt
void adc_int_clr(uint32_t* adc) {
	adc[0x00C/4] |= 0x01;
}

void spi_init(uint32_t* spi, uint8_t sph, uint8_t spo, uint8_t divisor) {
	uint8_t clock_mask;
	uint32_t* gpio_port;
	uint8_t gpio_pins;
	uint32_t alt_func;
	if (spi == SPI0) {
		clock_mask = 0x01;
		gpio_port = GPIO_A;
		gpio_pins = 0x3C;
	} else if (spi == SPI1) {
		clock_mask = 0x02;
		gpio_port = GPIO_D;
		gpio_pins = 0x0F;
	} else if (spi == SPI2) {
		clock_mask = 0x04;
		gpio_port = GPIO_B;
		gpio_pins = 0xF0;
	} else if (spi == SPI3) {
		clock_mask = 0x08;
		gpio_port = GPIO_D;
		gpio_pins = 0x0F;
	}
	SYS_CTL[0x61C/4] |= clock_mask;
	
	gpio_clock(gpio_port);
	gpio_afsel(gpio_port, gpio_pins, 1);
	
	
	alt_func = gpio_port[0x52C/4];		// read-modify-write alt. functions
	if (gpio_pins == 0x3C) {
		alt_func &= 0xFF0000FF;
		alt_func |= 0x00222200;
	} else if (gpio_pins == 0xF0) {
		alt_func &= 0x0000FFFF;
		alt_func |= 0x22220000;
	} else if (spi == SPI1) {
		alt_func &= 0xFFFF0000;
		alt_func |= 0x00002222;
	} else if (spi == SPI3) {
		alt_func &= 0xFFFF0000;
		alt_func |= 0x00001111;
	}
	gpio_port[0x52C/4] = alt_func;
	gpio_den(gpio_port, gpio_pins, 1);
	gpio_pur(gpio_port, gpio_pins & (gpio_pins << 1), 1);
	
	spi[0x004/4] = 0x00;		// disable SSI, set to master mode
	spi[0xFC8/4] = 0x00;		// use system clock
	spi[0x010/4] = divisor;		// set prescale divisor
	spi[0x000/4] = 0x0007 | spo | sph;	// freescale, no divisor, clock polarity and phase
	if (spo) {	// enable pull up resistor of clock idles high
		gpio_pur(gpio_port, (gpio_pins >> 3) & gpio_pins, 1);
	}
	spi[0x004/4] |= 0x02;		// enable SSI
}

uint8_t spi_rx_is_empty(uint32_t* spi) {
	return !(spi[0x00C/4] & 0x04);
}

uint8_t spi_tx_is_full(uint32_t* spi) {
	return !(spi[0x00C/4] & 0x02);
}

void spi_rw(uint32_t* spi, uint8_t* data, uint8_t size) {
	uint8_t i;
	for (i=0; i<size; ++i) {
		while (spi_tx_is_full(spi));
		spi[0x008/4] = data[i];
		while (spi_rx_is_empty(spi));
		data[i] = spi[0x008/4];
	}
}

void spi_r(uint32_t* spi, uint8_t* data, uint8_t size) {
	uint8_t i;
	for (i=0; i<size; ++i) {
		while (spi_tx_is_full(spi));
		spi[0x008/4] = data[i];
	}
}

void pwm_init(uint32_t* pwm, uint32_t clk_speed, uint16_t freq) {
	uint32_t* gpio_port;
	uint8_t gpio_pins;
	uint8_t enable_bit;
	SYS_CTL[0x100/4] |= 0x00100000;		// enable clock for PWM module
	if (pwm == PWM0) {
		gpio_port = GPIO_B;
		gpio_pins = 0x40;
		enable_bit = 0x01;
	} else if (pwm == PWM1) {
		gpio_port = GPIO_B;
		gpio_pins = 0x10;
		enable_bit = 0x04;
	} else if (pwm == PWM2) {
		gpio_port = GPIO_E;
		gpio_pins = 0x10;
		enable_bit = 0x10;
	} else if (pwm == PWM3) {
		gpio_port = GPIO_C;
		gpio_pins = 0x10;
		enable_bit = 0x40;
	}
	
	gpio_clock(gpio_port);
	gpio_afsel(gpio_port, gpio_pins, 1);
	gpio_den(gpio_port, gpio_pins, 1);
	
	if (gpio_pins == 0x10) {	// select PWM function on pins
		gpio_port[0x52C/4] = (gpio_port[0x52C/4] & 0xFFF0FFFF) | 0x00040000;
	} else if (gpio_pins == 0x40) {
		gpio_port[0x52C/4] = (gpio_port[0x52C/4] & 0xF0FFFFFF) | 0x04000000;
	}
	
	SYS_CTL[0x060/4] = (SYS_CTL[0x060/4] & 0xFFE1FFFF) | 0x00120000;	// divide sys_clock by 4
	
	pwm[0x000/4] = 0x00000000;	// turn PWM off
	pwm[0x020/4] = 0x000000C2;	// pwm A turns on when cmp. A down and off when 0 
	pwm[0x024/4] = 0x00000000;	// disable pwm B output
	pwm[0x010/4] = (clk_speed >> 2) / freq - 1;	// set frequency
	pwm[0x018/4] = 0;						// duty cycle default to 0%
	pwm[0x000/4] = 0x00000001;	// start PWM timers
	PWM_BASE[0x008/4] |= enable_bit; // route PWM to pin
}

void pwm_set_freq(uint32_t* pwm, uint32_t clk_speed, uint16_t freq) {
	float duty = pwm[0x018/4] / (float) pwm[0x010/4];	// duty cycle
	pwm[0x010/4] = (clk_speed >> 2) / freq - 1;			// new frequency
	pwm[0x018/4] = pwm[0x010/4] * duty;					// new cmpA value, preserving duty cycle
}

void pwm_set_duty(uint32_t* pwm, float duty) {
	if (duty == 1.0f || duty == -1.0f) {
		pwm[0x018/4] = pwm[0x010/4] - 1;
	} else {
		pwm[0x018/4] = pwm[0x010/4] * duty - 1;
	}
}

void msleep(uint32_t milliseconds) {
	uint32_t max, ms;
	for (ms = milliseconds; ms > 0; --ms) {
		for (max=4000; max > 0; --max);	// this loops for 1 ms
										// calculated from the disassembly
	}
}
