#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart.h"

#define APA102_START_FRAME 0x0
#define APA102_END_FRAME 0xFFFFFFFF

#define LED_D6 0
#define LED_D7 1
#define LED_D8 2

#define APA102_FRAME_HEADER (0b111 << 5)

#define MOSI (1 << PB3)
#define SCK (1 << PB5)

volatile uint32_t g_led_strip[5];

void spi_init(void)
{
	//setting clock at 1000khz since LEDS need something in 800-1200range
	//since the atmega is at 16 000khz we just divide by 16
	SPCR = (1 << SPR0);

	//setting master mode
	SPCR |= (1 << MSTR) | (1 << SPE);

	//setting falling leading edge and rising trailing edge as
	//seen in the APA102 datasheet
	// SPCR |= (1 << CPOL) | (1 << CPHA);

	//nothing to do to setup MSB first a requested in led APA102 datasheet
	//since it is default byte order in SPI

	g_led_strip[0] = APA102_START_FRAME;
	g_led_strip[4] = APA102_END_FRAME;
}

void spi_send_byte(uint8_t byte)
{
	uart_printstr("sending byte:");
	print_hex_value(byte);
	uart_printstr("\r\n");
	SPDR = byte;

	while (!(SPSR & (1<<SPIF)));
}

void spi_send_buffer(uint8_t * buffer, uint8_t size)
{
	for(uint8_t i = 0; i < size; i++)
		spi_send_byte(buffer[i]);
	SPCR &= ~(1 << SPE);
}

void spi_set_led(uint8_t led_id, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
	//an APA102 FRAME IS
	//[111][brightness] [BLUE]	[GREEN]	[RED]
	//		8 bits		 		8 bits each

	g_led_strip[1 + led_id] = ((uint32_t)brightness | APA102_FRAME_HEADER) << 24;
	g_led_strip[1 + led_id] |= ((uint32_t)b << 16) | ((uint32_t)g << 8) | (uint32_t)r;
}

void update_strip(void)
{
	spi_send_buffer((uint8_t *)g_led_strip, 5 * 4);
}

int main()
{
	DDRB = 1 << PB3 | 1 << PB5 | 1 << PB2;


	spi_init();
	spi_set_led(LED_D6, 255, 0, 0, 1);
	spi_set_led(LED_D7, 0, 0, 0, 0);
	spi_set_led(LED_D8, 0, 0, 0, 0);
	update_strip();
}
