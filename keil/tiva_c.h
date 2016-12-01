#ifndef TIVA_C_H
#define TIVA_C_H

#include <stdint.h>

extern uint32_t* SYS_CTL;
extern uint32_t* CORE_P;

extern uint8_t CLK_MOSC;
extern uint8_t CLK_PIOSC;
extern uint8_t CLK_PLL_ON;
extern uint8_t CLK_PLL_OFF;

extern uint32_t* GPIO_A; 
extern uint32_t* GPIO_B;
extern uint32_t* GPIO_C;
extern uint32_t* GPIO_D;
extern uint32_t* GPIO_E; 
extern uint32_t* GPIO_F;

extern uint8_t GPIO_IN;
extern uint8_t GPIO_OUT;
extern uint8_t GPIO_TRI;
extern uint8_t GPIO_PUR;
extern uint8_t GPIO_PDR;
extern uint8_t GPIO_ODR;
extern uint8_t GPIO_DEN;
extern uint8_t GPIO_NONE;

extern uint8_t GPIO_RISING;
extern uint8_t GPIO_FALLING;
extern uint8_t GPIO_BOTH;

extern uint32_t* UART0;
extern uint32_t* UART1;
extern uint32_t* UART2;
extern uint32_t* UART3;
extern uint32_t* UART4;
extern uint32_t* UART5;
extern uint32_t* UART6;
extern uint32_t* UART7;

extern uint32_t* I2C_0;
extern uint32_t* I2C_1;
extern uint32_t* I2C_2;
extern uint32_t* I2C_3;

extern uint32_t I2C_100k;
extern uint32_t I2C_400k;
extern uint32_t I2C_1000k;
extern uint32_t I2C_HIGHSPEED;

extern uint32_t* TIMER32_0;
extern uint32_t* TIMER32_1;
extern uint32_t* TIMER32_2;
extern uint32_t* TIMER32_3;
extern uint32_t* TIMER32_4;
extern uint32_t* TIMER32_5;

extern uint8_t TIMER_ONESHOT;
extern uint8_t TIMER_PERIODIC;

extern uint32_t* ADC0;
extern uint32_t* ADC1;

extern uint32_t* SPI0;
extern uint32_t* SPI1;
extern uint32_t* SPI2;
extern uint32_t* SPI3;

extern uint8_t SPI_ACTIVE_LOW;
extern uint8_t SPI_ACTIVE_HIGH;
extern uint8_t SPI_CLK_FIRST;
extern uint8_t SPI_CLK_SECOND;

extern uint32_t* PWM_BASE;
extern uint32_t* PWM0;
extern uint32_t* PWM1;
extern uint32_t* PWM2;
extern uint32_t* PWM3;

void sys_clock(uint8_t src, uint8_t pll, uint8_t sysdiv);

void nvic_int_en(uint8_t interrupt);
void nvic_set_pri(uint8_t interrupt, uint8_t priority);

void systick_init(uint32_t reload);
uint32_t systick_current(void);

void gpio_init(uint32_t* port, uint8_t pins, uint8_t dir, uint8_t state);
void gpio_int_edge_en(uint32_t* port, uint8_t pins, uint8_t edge);
void gpio_int_edge_clr(uint32_t* port, uint8_t pins);
void gpio_clock(uint32_t* port);
void gpio_unlock(uint32_t* port, uint8_t pins);
void gpio_pur(uint32_t* port, uint8_t pins, uint8_t state);
void gpio_pdr(uint32_t* port, uint8_t pins, uint8_t state);
void gpio_odr(uint32_t* port, uint8_t pins, uint8_t state);
void gpio_den(uint32_t* port, uint8_t pins, uint8_t state);
void gpio_afsel(uint32_t* port, uint8_t pins, uint8_t state);
void gpio_write(uint32_t* port, uint8_t pin, uint8_t value);
uint8_t gpio_read(uint32_t* port, uint8_t pin);

void uart_init(uint32_t* uart, uint32_t baud, uint32_t sys_clock);
void uart_send(uint32_t* uart, uint8_t data);
void uart_receive(uint32_t* uart, uint8_t* data);
uint8_t uart_rx_is_empty(uint32_t* uart);
uint8_t uart_tx_is_full(uint32_t* uart);
void uart_send_stream(uint32_t* uart, unsigned char* str);
void uart_rx_int(uint32_t* uart);
int uprintf(uint32_t* uart, const char* format, ...);

void timer_init(uint32_t* timer, uint32_t reload, uint8_t mode);
void timer_start(uint32_t* timer);
void timer_stop(uint32_t* timer);
uint32_t timer_value(uint32_t* timer);
void timer_timeout_int_en(uint32_t* timer);
void timer_timeout_int_clr(uint32_t* timer);
uint8_t timer_expired(uint32_t* timer);

void i2c_init(uint32_t* i2c, uint32_t sys_clock, uint32_t speed);
uint8_t i2c_is_busy(uint32_t* i2c);
void i2c_write(uint32_t* i2c, uint8_t address, uint8_t* data, uint8_t size, uint8_t HS);
void i2c_read(uint32_t* i2c, uint8_t address, uint8_t s_address, uint8_t* data, uint32_t size);

uint32_t* bitband(uint32_t* address, uint8_t bit);
void msleep(uint32_t milliseconds);

void adc_init(uint32_t* adc, uint32_t* timer);
uint16_t adc_read(uint32_t* adc);
void adc_int_clr(uint32_t* adc);

void spi_init(uint32_t* spi, uint8_t sph, uint8_t spo, uint8_t divisor);
uint8_t spi_rx_is_empty(uint32_t* spi);
uint8_t spi_tx_is_full(uint32_t* spi);
void spi_rw(uint32_t* spi, uint8_t* data, uint8_t size);
void spi_r(uint32_t* spi, uint8_t* data, uint8_t size);

void pwm_init(uint32_t* pwm, uint32_t clk_speed, uint16_t freq);
void pwm_set_freq(uint32_t* pwm, uint32_t clk_speed, uint16_t freq);
void pwm_set_duty(uint32_t* pwm, float duty);

#endif
