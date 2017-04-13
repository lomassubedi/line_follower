/*
 * LineFollower.c
 *
 * Created: 4/4/2017 04:12:36 PM
 * Author : LOMAS
 */ 

#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include <stdio.h>

#define BAUD 9600
#define UBBR_VAL (((F_CPU)/(BAUD*16UL)) - 1)            // Calculate value for UBBR

#define MOTOR_PORT PORTD
#define MOTOR_PORT_DIRN DDRD

#define MOTOR_RIGHT_IN1 PORTD0
#define MOTOR_RIGHT_IN2 PORTD1

#define MOTOR_LEFT_IN1 PORTD2
#define MOTOR_LEFT_IN2 PORTD3

void init_ios() {
	MOTOR_PORT_DIRN = 0xFF;
	MOTOR_PORT = 0x00;
}

void init_UART(void) {
	UBRRL = UBBR_VAL;                           // Write to Lower UBBR register
	UBRRH = (UBBR_VAL>>8);                      // Write to Higher UBBR register
	UCSRB|= (1<<TXEN)|(1<<RXEN);				// enable receiver and transmitter
	UCSRC|= (1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1);   // 8bit data format, No parity, One stop bit
}

int uart_putchar(char c, FILE *stream) {
	if (c == '\n')
	uart_putchar('\r', stream);
	while(!(UCSRA & (1 << UDRE))); // Loop until UDRE is set
	UDR = c;
	return 0;
}

void init_adc(void) {
	ADCSRA |= (1<<ADEN);					// Enable ADC of ATmega16
	ADCSRA |= ((1<<ADPS1)|(1<<ADPS2));		// Set 12MHz/64 = 187KHz which is <200KHz	|
	ADMUX |= ((1<<REFS0));					// AREF = AVcc
	ADMUX &= ~(1<<ADLAR);					// Left Justified Results in Higher ADC register
}

unsigned int read_adc(unsigned char channel) {
	channel = channel & 0b00000111;		// channel must be between 0 to 7
	ADMUX |= channel;					// selecting channel
	
	ADCSRA |= (1<<ADSC);				// start conversion
	while(!(ADCSRA & (1 << ADIF)));		// waiting for ADIF
	ADCSRA|=(1<<ADIF);					// clearing of ADIF, it is done by writing 1 to it
	ADMUX &= 0b11111000;
	return ADC;
}
uint8_t check_sensor(uint16_t sensor_val) {
	if( sensor_val > 280) 
		return 1;
	else
		return 0;
}

void robot_forward() {
	MOTOR_PORT |= (1 << MOTOR_RIGHT_IN2) | (1 << MOTOR_LEFT_IN2);
	MOTOR_PORT &= ~((1 << MOTOR_RIGHT_IN1) | (1 << MOTOR_LEFT_IN1));
}

void robot_left() {
	MOTOR_PORT |= (1 << MOTOR_RIGHT_IN2);
	MOTOR_PORT &= ~((1 << MOTOR_RIGHT_IN1) | (1 << MOTOR_LEFT_IN1) | (1 << MOTOR_LEFT_IN2));
}

void robot_right() {
	MOTOR_PORT |= (1 << MOTOR_LEFT_IN2);
	MOTOR_PORT &= ~((1 << MOTOR_RIGHT_IN1) | (1 << MOTOR_LEFT_IN1) | (1 << MOTOR_RIGHT_IN2));
}

void robot_stop() {
	MOTOR_PORT = 0x00;	
}

int main(void) {

    uint16_t left_sensor = 0, mid_sensor_l = 0, mid_sensor_r = 0, right_sensor = 0;

	/*init_UART();*/
	init_adc();
	init_ios();
	// Setup for pipelining UART data to C standard IO library making printf() work
    static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &mystdout;	
	
    while (1) {
		left_sensor = read_adc(0);
		mid_sensor_l = read_adc(1);
		mid_sensor_r = read_adc(2);
		right_sensor = read_adc(3);

		if(check_sensor(mid_sensor_l) && check_sensor(mid_sensor_r) && (!check_sensor(left_sensor)) && (!check_sensor(right_sensor))) robot_forward();
		else if(check_sensor(left_sensor) && check_sensor(mid_sensor_l) && (!check_sensor(mid_sensor_r)) && (!check_sensor(right_sensor))) robot_right();
		else if(check_sensor(right_sensor) && check_sensor(mid_sensor_r) && (!check_sensor(mid_sensor_l)) && (!check_sensor(left_sensor))) robot_left();
		else robot_stop();
	}
	return 0;
}


