#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "../uart.h"

#define RV1_PIN (0b0000)
#define LDR_PIN (0b0001)
#define NTC_PIN (0b0010)

#define APA102_START_FRAME 0x0
#define APA102_END_FRAME 0xFFFFFFFF

#define LED_D6 0
#define LED_D7 1
#define LED_D8 2
#define MAX_LED LED_D8
#define MIN_LED LED_D6

#define APA102_FRAME_HEADER ((uint8_t)0b11100000)
#define BLUE_OFFSET 1
#define GREEN_OFFSET 2
#define RED_OFFSET 3

#define HEX_CHARS "0123456789ABCDEF"

uint8_t g_led_strip[5 * 4];

volatile uint8_t last_measure = 0;
volatile uint8_t g_pos = 0;
volatile uint8_t unicorns_launched = 0;

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

	g_led_strip[0] = 0x0; //setting start frame
	g_led_strip[1] = 0x0;
	g_led_strip[2] = 0x0;
	g_led_strip[3] = 0x0;
	g_led_strip[(4 * 4) +0] = 0xFF; //setting end frame
	g_led_strip[(4 * 4) +1] = 0xFF;
	g_led_strip[(4 * 4) +2] = 0xFF;
	g_led_strip[(4 * 4) +3] = 0xFF;
}

void spi_send_byte(uint8_t byte)
{
	SPDR = byte;

	while (!(SPSR & (1<<SPIF)));
}

void spi_send_buffer(uint8_t * buffer, uint8_t size)
{
	for(uint8_t i = 0; i < size; i++)
		spi_send_byte(buffer[i]);
}

void spi_set_led(uint8_t led_id, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
	//an APA102 FRAME IS
	//[111][brightness] [BLUE]	[GREEN]	[RED]
	//		8 bits		 		8 bits each

	g_led_strip[(1 + led_id) * 4] = brightness | APA102_FRAME_HEADER;
	g_led_strip[(1 + led_id) * 4 + BLUE_OFFSET] = b;
	g_led_strip[(1 + led_id) * 4 + GREEN_OFFSET] = g;
	g_led_strip[(1 + led_id) * 4 + RED_OFFSET] = r;
}

void update_strip(void)
{
	spi_send_buffer(g_led_strip, 5 * 4);
}

void clear_strip()
{
	spi_set_led(LED_D6, 0, 0, 0, 0);
	spi_set_led(LED_D7, 0, 0, 0, 0);
	spi_set_led(LED_D8, 0, 0, 0, 0);
}

int8_t ft_strcmp(char *str1, char *str2)
{
	while (*str1 && *str2)
	{
		if (*str1 != *str2)
			return (*str1 - *str2);
		str1++;
		str2++;
	}
	return *str1 - *str2;
}

uint8_t is_inside(char c, char * str)
{
	while(*str)
	{
		if (c == *str)
			return 1;
		str++;
	}
	return 0;
}

uint8_t is_led_id(char *str)
{
	if (str[0] != 'D')
		return 0;
	if (str[1] < '6' || str[1] > '8')
		return 0;
	if (str[2] != '\0')
		return 0;
	return 1;
}

uint8_t is_rgb_format(char * buffer)
{
	uint8_t i = 0;

	if (buffer[0] != '#')
		return 0;
	while (i < 6 && buffer[i + 1] != '\0')
	{
		if (is_inside(buffer[i + 1], HEX_CHARS) == 0)
			return 0;
		i++;
	}
	if (i == 6 && is_led_id(buffer + 7))
		return (1);
	return (0);
}

uint8_t str_find(char *str, char c)
{
	for(uint8_t i = 0; str[i]; i++)
	{
		if (str[i] == c)
			return i;
	}
	return 0;
}

uint32_t atoi_base(char *str, char *base, uint8_t base_length)
{
	uint32_t ret = 0;

	size_t i = 0;
	while(i < 6 && str[i])
	{
		ret *= base_length;
		ret += str_find(base, str[i]);
		i++;
	}
	return ret;
}

void str_to_rgb(char *str)
{
	uint32_t rgb = atoi_base(str + 1, HEX_CHARS, 16);

	spi_set_led(str[8] - '6', (rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF, 1);
	update_strip();
}

void wheel(uint8_t pos, uint8_t led_id)
{
	// pos = 255 - pos;
	if (pos < 85)
	{
		spi_set_led(led_id, 255 - pos * 3, 0, pos * 3, 1);
	}
	else if (pos < 170)
	{
		pos = pos - 85;
		spi_set_led(led_id, 0, pos * 3, 255 - pos * 3, 1);
	}
	else
	{
		pos = pos - 170;
		spi_set_led(led_id, pos * 3, 255 - pos * 3, 0, 1);
	}
}

ISR(TIMER0_OVF_vect)
{
	g_pos++;
	wheel(g_pos + 60, LED_D6);
	wheel(g_pos + 30, LED_D7);
	wheel(g_pos, LED_D8);
	update_strip();
}

void init_timer()
{
	TCCR0A = 0;

	//setting normal mode with 256 prescaler
	TCCR0B = (1 << CS02) | (1 << CS00);

	TIMSK0 = (1 << TOIE0);
}

void launch_unicorns(void)
{
	unicorns_launched = 1;
	sei();
}

int main()
{
	DDRB = 1 << PB3 | 1 << PB5 | 1 << PB2;


	spi_init();
	uart_init();
	init_timer();

	spi_set_led(LED_D6, 0, 0, 0, 0);
	spi_set_led(LED_D7, 0, 0, 0, 0);
	spi_set_led(LED_D8, 0, 0, 0, 0);
	update_strip();

	char buffer[BUFFER_SIZE];
	uint8_t index = 0;

	uart_printstr("Emter RGBDX value in hex #RRGGBBDX format ( dx is the led you want to select)\r\n");
	for(;;)
	{
		uart_receive_word(buffer, &index, 0);
		if (ft_strcmp(buffer, "#FULLRAINBOW") == 0)
			launch_unicorns();
		else if (is_rgb_format(buffer) == 0)
			uart_printstr("Wrong format, #RRGGBBDX\r\n");
		else
		{
			if (unicorns_launched == 1)
			{
				cli();
				unicorns_launched = 0;
				clear_strip();
			}
			str_to_rgb(buffer);
		}
	}
}
