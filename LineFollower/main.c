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

#define MOTOR_LEFT_PORT			PORTD
#define MOTOR_LEFT_PORT_DIRN	DDRD
#define MOTOR_LEFT_PIN			PORTD7

#define MOTOR_RIGHT_PORT		PORTB
#define MOTOR_RIGHT_PORT_DIRN	DDRB
#define MOTOR_RIGHT_PIN			PORTB3


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
void pwm_motor_left(uint8_t duty) {
	OCR2 = duty;
	return;
}

void pwm_motor_right(uint8_t duty) {
	OCR0 = duty;
	return;
}

void init_pwm(void) {
	MOTOR_RIGHT_PORT_DIRN |=(1 << MOTOR_RIGHT_PIN);
	TCCR0 = 0x62;			
	
	MOTOR_LEFT_PORT_DIRN |=(1 << MOTOR_LEFT_PIN);		
	TCCR2 = 0x62;
}

uint8_t check_sensor(uint16_t sensor_val) {
	if( sensor_val > 280) 
		return 1;
	else
		return 0;
}

int main(void) {

    uint16_t left_sensor = 0, mid_sensor_l = 0, mid_sensor_r = 0, right_sensor = 0;

	init_UART();
	init_adc();
	init_pwm();
	pwm_motor_left(128);
	pwm_motor_right(128);
	/*init_ios();*/
	
	// Setup for pipelining UART data to C standard IO library making printf() work
    static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &mystdout;	
	uint16_t sensor_raw[6];
	
	char error_weight[] = {-3, -2, -1, 1, 2, 3}; 
		
	float error = 0.0;
	float pos_current = 0.0;
	float pos_target = 0.0;
	
	// Tunning parameters 
	float Kp = 0.0;
	
	
	// P, I and D values 
	float P = 0.0;
	
	
	
    while (1) {
		for(uint8_t i = 0; i < 6; i++) {
			sensor_raw[i] = read_adc(i);
		}
		printf("Sensor 0 : %d\tSensor 1 : %d\tSensor 2 : %d\tSensor 3 : %d\tSensor 4 : %d\tSensor 5 : %d\n", sensor_raw[0], sensor_raw[1], sensor_raw[2], sensor_raw[3], sensor_raw[4], sensor_raw[5]);
		_delay_ms(500);
		
// 		left_sensor = read_adc(0);
// 		mid_sensor_l = read_adc(1);
// 		mid_sensor_r = read_adc(2);
// 		right_sensor = read_adc(3);

// 		if(check_sensor(mid_sensor_l) && check_sensor(mid_sensor_r) && (!check_sensor(left_sensor)) && (!check_sensor(right_sensor))) robot_forward();
// 		else if(check_sensor(left_sensor) && check_sensor(mid_sensor_l) && (!check_sensor(mid_sensor_r)) && (!check_sensor(right_sensor))) robot_right();
// 		else if(check_sensor(right_sensor) && check_sensor(mid_sensor_r) && (!check_sensor(mid_sensor_l)) && (!check_sensor(left_sensor))) robot_left();
// 		else robot_stop();
	}
	return 0;
}


