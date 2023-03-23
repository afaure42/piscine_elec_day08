#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

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

uint8_t g_led_strip[5 * 4];

volatile uint8_t last_measure = 0;
volatile uint8_t selected_led = 0;
volatile uint8_t last_red = 0;
volatile uint8_t last_green = 0;
volatile uint8_t last_blue = 0;
volatile uint8_t selected_color = 0;

volatile uint8_t g_switch1_counter = 0;

ISR(INT0_vect)
{
		g_switch1_counter++;
	if(g_switch1_counter & 1)
	{
		selected_color++;
		if (selected_color > 2)
			selected_color = 0;
	}
	_delay_ms(1);
	EIFR = 1 << INTF0;
}

void adc_init(void)
{
	//setting prescaler at 128 because 16 000 000 / 128 = 125 000 wich is in the range of the ADC
	//and enabling auto trigger
	ADCSRA = (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2) | (1 << ADATE);
	// ADCSRB = (1 << ADTS2) | (1 << ADTS0); //setting auto trigger on timer 0 compare match b

	ADCSRA |= (1 << ADIE); //enabling interrupt
	ADCSRA |= 1 << ADEN; //enabling ADC

	ADMUX = (1 << REFS0); //setting reference voltage as AREF pin / AVcc
	ADMUX |= (1 << ADLAR); //enabling left adjustment in data registers ( since i only want 8 bit of precision)
	ADMUX |= (RV1_PIN << MUX0); //seetting RV1 as input
	
	//leaving MUX selection bits as false in ADMUX since i only want ADC0 as input
}

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

void init_timer(void)
{
	//setting up timer 0 to trigger an ADC measurement every ms more or less
	//setting 64 prescaler with normal mode
	TCCR0A = 0;
	TCCR0B = (1 << CS01) | (1 << CS00);

	OCR0B =  0xFF;
}

ISR(ADC_vect)
{
	uint8_t temp = ADCH;
	if (temp != last_measure)
	{
		last_measure = temp;
		if (selected_color == 0){
			last_red = last_measure;
		} else if (selected_color == 1){
			last_green = last_measure;
		} else
			last_blue = last_measure;
		spi_set_led(selected_led, last_red, last_green, last_blue, 1);
		update_strip();
	}
}


int main()
{
	DDRB = 1 << PB3 | 1 << PB5 | 1 << PB2;


	spi_init();
	adc_init();

	spi_set_led(LED_D6, 0, 0, 0, 0);
	spi_set_led(LED_D7, 0, 0, 0, 0);
	spi_set_led(LED_D8, 0, 0, 0, 0);
	update_strip();

	sei();
	ADCSRA |= (1 << ADSC);
	for(;;)
	{
	}
}
