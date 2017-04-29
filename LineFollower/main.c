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


#define MOTOR_LEFT_PORT_DIRN	DDRD
#define MOTOR_LEFT_PIN			PORTD7

#define MOTOR_RIGHT_PORT_DIRN	DDRB
#define MOTOR_RIGHT_PIN			PORTB3

#define SENSOR_TRACK_TRUE_VAL	850


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

void init_timer1() {
	
}
uint8_t sensor_schmmit_trigger(uint16_t sensor_val) {
	if( sensor_val > SENSOR_TRACK_TRUE_VAL) 
		return 1;
	else
		return 0;
}

int main(void) {    
	
	init_UART();
	init_adc();
	init_pwm();
	pwm_motor_left(128);
	pwm_motor_right(128);
	
	// Setup for pipelining UART data to C standard IO library making printf() work
    static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &mystdout;	
	
	uint16_t sensor_raw[6];
	uint8_t sensor_digit[6];
	
	char error_weight[] = {-3, -2, -1, 1, 2, 3}; 
		
	int8_t error = 0;
	float pos_current = 0.0;
	float pos_target = 0.0;
	
	// Tunning parameters (gain) 
	float Kp = 1.7;
	float Kd = 3;
	float Ki = 0.5;
	
	// P, I and D values 
	float P = 0.0;
	float D = 0.0;
	float I = 0.0;
		
	// Correction constants
	uint16_t multiplier = 15;
	float last_error = 0;
	int16_t correction = 0;
	uint8_t basePWM = 200;
	
	int16_t pwm_left_motor = 0;
	int16_t pwm_right_motor = 0;
	uint8_t sum_sensor_digit = 0;
	
    while (1) {
		error = 0;
		
		for(uint8_t i = 0; i < 6; i++) {
			sensor_raw[i] = read_adc(i);			
		}
		
		for(uint8_t i = 0; i < 6; i++) {
			if(sensor_schmmit_trigger(sensor_raw[i]))
				sensor_digit[i] = 1;				
			else
				sensor_digit[i] = 0;	
			sum_sensor_digit += sensor_digit[i];
		}
		
		for(uint8_t i = 0; i < 6; i++) {
			if(sensor_digit[i])
				error += error_weight[i];			
		}
		
		// Calculation of Proportional Component
		P = Kp * error;
				
		// Calculation of Integral Component
		I = Ki * (error + last_error);
		
		// Calculation of Derivative Component
		D = Kd * (error - last_error);
		
		
		// ----------------- PD --------------------
		/*correction = ((int)P  + (int)D) * multiplier;*/
		
		// ----------------- PI --------------------
		/*correction = ((int)P  + (int)I) * multiplier;*/		
		
		// ----------------- PID --------------------
		correction = ((int)P  + (int)I + (int)D) * multiplier;		
		
		// ----- Save last error value -------- //
		last_error = error;
		
		
		// ----- Apply correction to motors ----- //
		
		if(correction >= 255) correction = 255;
		
		if(correction <= -255) correction = -255;
		
		pwm_left_motor = basePWM + correction;
		pwm_right_motor = basePWM - correction;
		
		
		if(pwm_left_motor > 255) pwm_left_motor = 255;
		if(pwm_right_motor > 255) pwm_right_motor = 255;
		
		if(pwm_left_motor < 0) pwm_left_motor = 0;
		if(pwm_right_motor < 0) pwm_right_motor = 0;
		
		if(sum_sensor_digit) {
			pwm_motor_left(pwm_left_motor);
			pwm_motor_right(pwm_right_motor);								
		} else {
			pwm_motor_left(128);
			pwm_motor_right(128);
		}
		sum_sensor_digit = 0;				
				
// 		printf("----------------------------------------------------------------------------------------------------------\n");
// 		printf("Sensor 0 : %d\tSensor 1 : %d\tSensor 2 : %d\tSensor 3 : %d\tSensor 4 : %d\tSensor 5 : %d\n", sensor_raw[0], sensor_raw[1], sensor_raw[2], sensor_raw[3], sensor_raw[4], sensor_raw[5]);
// 		printf("Error : %d\n", error);	
// 		printf("PWM Left Motor : %d\t PWM Right Motor : %d\n", pwm_left_motor, pwm_right_motor);	
		/*_delay_ms(500);*/
		
		
	}
	return 0;
}

