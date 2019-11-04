/*
 * Cicada.c
 *
 * Created: 25.08.2019 12:45:42
 * Author : Burnmind
 */ 

#define F_CPU 1000000UL
#include <avr/io.h>
#include <avr/interrupt.h>

#define ciclesInSecond 200
#define sensorSecondsOfSuspension 30

#define false 0
#define true !false

void startTimer();
void stopTimer();
void initTimerOne();
void initMotionSensor();

int seconds = 0;
int currentCicle = 0;

int simpeSize = 0;
int pointer0 = 0;
int pointer1 = 100;
int pointer2 = 200;

char moutionSensorStop = false;

unsigned char voice [] = {0x64, 0x61, 0x74, 0x61, 0xe2, 0x5e, 0x0, 0x0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x7f, 0x81, 0x80, 0x80, 0x80, 0x80, 0x80, 0x81, 0x81, 0x7f, 0x80, 0x81, 0x80, 0x81, 0x80, 0x7f, 0x81, 0x81, 0x80, 0x80, 0x7f, 0x81, 0x81, 0x80, 0x80, 0x7f, 0x82, 0x81, 0x80, 0x7e, 0x81, 0x83, 0x7f, 0x7d, 0x83, 0x81, 0x7f, 0x84, 0x7f, 0x7e, 0x81, 0x83, 0x80, 0x7c, 0x81, 0x85, 0x81, 0x7d, 0x81, 0x82, 0x7e, 0x83, 0x79, 0x83, 0x84, 0x81, 0x7c, 0x80, 0x85, 0x7c, 0x83, 0x7f, 0x7c, 0x89, 0x7f, 0x7e, 0x80, 0x81, 0x84, 0x82, 0x78, 0x83, 0x85, 0x81, 0x7d, 0x80, 0x83, 0x80, 0x7f, 0x82, 0x7e, 0x84, 0x7f, 0x81, 0x7f, 0x81, 0x7f, 0x84, 0x81, 0x7d, 0x7f, 0x83, 0x7f, 0x81, 0x7e, 0x81, 0x84, 0x7e, 0x7d, 0x83, 0x80, 0x81, 0x83, 0x7d, 0x7f, 0x84, 0x80, 0x7f, 0x80, 0x82, 0x82, 0x7d, 0x7e, 0x86, 0x7e, 0x7e, 0x78, 0x94, 0x79, 0x79, 0x88, 0x7e, 0x84, 0x7c, 0x7f, 0x82, 0x84, 0x81, 0x7d, 0x7e, 0x86, 0x86, 0x74, 0x83, 0x86, 0x80, 0x7d, 0x80, 0x81, 0x82, 0x7f, 0x81, 0x82, 0x81, 0x80, 0x81, 0x7d, 0x87, 0x7f, 0x7e, 0x7e, 0x87, 0x82, 0x76, 0x87, 0x82, 0x7d, 0x8d, 0x6f, 0x83, 0x8a, 0x7a, 0x7f, 0x85, 0x88, 0x75, 0x80, 0x89, 0x79, 0x86, 0x7c, 0x83, 0x85, 0x7e, 0x82, 0x7d, 0x80, 0x82, 0x85, 0x7e, 0x7f, 0x7c, 0x86, 0x87, 0x76, 0x80, 0x85, 0x82, 0x7c, 0x8b, 0x7b, 0x78, 0x89, 0x89, 0x72, 0x80, 0x86, 0x82, 0x81, 0x81, 0x7f, 0x7b, 0x89, 0x83, 0x7d, 0x7f, 0x83, 0x86, 0x7a, 0x7f, 0x86, 0x7c, 0x81, 0x84, 0x85, 0x79, 0x82, 0x7e, 0x83, 0x82, 0x80, 0x80, 0x8c, 0x74, 0x7d, 0x91, 0x74, 0x7e, 0x7e, 0x96, 0x6e, 0x83, 0x85, 0x7b, 0x8d, 0x7d, 0x7a, 0x81, 0x86, 0x7e, 0x85, 0x79, 0x85, 0x87, 0x77, 0x84, 0x88, 0x7f, 0x77, 0x84, 0x8d, 0x7b, 0x7b, 0x84, 0x86, 0x7f, 0x7f, 0x81, 0x7f, 0x7f, 0x81, 0x83, 0x81, 0x84, 0x7e, 0x71, 0x95, 0x80, 0x6d, 0x8f, 0x87, 0x6e, 0x86, 0x88, 0x74, 0x91, 0x7b, 0x7a, 0x86, 0x87, 0x78, 0x7f, 0x85, 0x80, 0x7b, 0x81, 0x87, 0x82, 0x76, 0x85, 0x89, 0x7b, 0x7f, 0x7f, 0x87, 0x7e, 0x84, 0x89, 0x63, 0xa2, 0x8b, 0x6a, 0x6e, 0x8b, 0x9e, 0x78, 0x67, 0x85, 0x8c, 0x98, 0x68, 0x76, 0x83, 0x94, 0x81, 0x73, 0x7a, 0x8b, 0x87, 0x7d, 0x78, 0x82, 0x89, 0x80, 0x77, 0x89, 0x83, 0x7f, 0x7b, 0x84, 0x88, 0x80, 0x75, 0x84, 0x86, 0x7f, 0x81, 0x7e, 0x7f, 0x81, 0x9a, 0x71, 0x4d, 0xac, 0x99, 0x5b, 0x8c, 0xa0, 0x5c, 0x84, 0x95, 0x73, 0x77, 0x93, 0x7d, 0x79, 0x8e, 0x80, 0x6f, 0x90, 0x80, 0x7c, 0x88, 0x7e, 0x74, 0x8d, 0x88, 0x67, 0x95, 0x9a, 0x56, 0x79, 0xa6, 0x84, 0x56, 0x86, 0x96, 0x92, 0x84, 0x50, 0x95, 0x93, 0x83, 0x5d, 0x8a, 0x92, 0x7a, 0x80, 0x7a, 0x82, 0x8e, 0x77, 0x79, 0x89, 0x8e, 0x73, 0x72, 0x90, 0x8c, 0x71, 0x83, 0x7e, 0x84, 0x88, 0x84, 0x6d, 0x87, 0x8e, 0x8a, 0x72, 0x5b, 0xba, 0x75, 0x6a, 0x97, 0x68, 0x8a, 0x95, 0x71, 0x71, 0x85, 0x94, 0x7f, 0x82, 0x81, 0x69, 0xa7, 0x76, 0x6d, 0x8d, 0x79, 0x90, 0x7c, 0x7a, 0xa0, 0x40, 0xa9, 0x9d, 0x48, 0x70, 0x9e, 0xc2, 0x3a, 0x68, 0x9e, 0x98, 0x7f, 0x5b, 0x7a, 0xac, 0x7e, 0x67, 0x82, 0x96, 0x7a, 0x79, 0x7e, 0x88, 0x89, 0x73, 0x76, 0x98, 0x88, 0x68, 0x7e, 0x91, 0x80, 0x76, 0x8a, 0x75, 0x83, 0x8a, 0x79, 0x7d, 0x85, 0x81, 0x7d, 0x7a, 0xbf, 0x35, 0x89, 0xbf, 0x40, 0x7a, 0x9b, 0x6f, 0x81, 0x98, 0x79, 0x63, 0xa3, 0x84, 0x5d, 0xa0, 0x79, 0x64, 0xa0, 0x7a, 0x70, 0x8f, 0x88, 0x67, 0x8d, 0x91, 0x63, 0x7c, 0xa4, 0x89, 0x47, 0x84, 0xb1, 0x8d, 0x47, 0x84, 0xab, 0x8a, 0x6c, 0x50, 0xb6, 0x89, 0x72, 0x5f, 0xa1, 0x88, 0x6b, 0x80, 0x8f, 0x80, 0x73, 0x80, 0x83, 0x92, 0x76, 0x68, 0x8f, 0x97, 0x73, 0x72, 0x86, 0x83, 0x90, 0x77, 0x71, 0x84, 0x96, 0x75, 0x7f, 0x78, 0x66, 0xc0, 0x63, 0x5e, 0xb0, 0x67, 0x79, 0x97, 0x6f, 0x74, 0x92, 0x8e, 0x6b, 0x86, 0x86, 0x76, 0x9a, 0x6c, 0x7a, 0x8f, 0x76, 0x86, 0x77, 0x87, 0x7d, 0x82, 0x78, 0xa7, 0x93, 0x2d, 0x98, 0xbb, 0x7a, 0x36, 0x93, 0xac, 0x89, 0x51, 0x68, 0xb7, 0x87, 0x64, 0x70, 0xa2, 0x7a, 0x7c, 0x6d, 0x92, 0x81, 0x71, 0x79, 0x9c, 0x7c, 0x67, 0x92, 0x87, 0x7a, 0x81, 0x7b, 0x75, 0x93, 0x89, 0x63, 0x82, 0x92, 0x7d, 0x76, 0x79, 0xa8, 0x57, 0x67, 0xa2, 0xa8, 0x3a, 0x95, 0xb4, 0x3d, 0xa4, 0x8e, 0x4c, 0x93, 0x9a, 0x64, 0x79, 0x9e, 0x68, 0x85, 0x98, 0x60, 0x83, 0x9e, 0x68, 0x74, 0x98, 0x7c, 0x64, 0xa8, 0x78, 0x4f, 0xa5, 0xa2, 0x5c, 0x65, 0x9b, 0xa2, 0x7f, 0x50, 0x7c, 0xba, 0x65, 0x73, 0x73, 0x97, 0x81, 0x79, 0x7c, 0x8f, 0x71, 0x81, 0x8a, 0x73, 0x82, 0x7f, 0x85, 0x7a, 0x89, 0x78, 0x7d, 0x88, 0x7f, 0x80, 0x83, 0x78, 0x8a, 0x82, 0x7e, 0x7f, 0x59, 0xac, 0x9b, 0x34, 0xab, 0x83, 0x67, 0x98, 0x77, 0x67, 0x8f, 0x88, 0x72, 0x81, 0x91, 0x63, 0x9d, 0x87, 0x5a, 0x99, 0x7d, 0x73, 0x91, 0x75, 0x7f, 0x8c, 0x42, 0xc2, 0x7f, 0x38, 0x96, 0xbc, 0x84, 0x1e, 0xa8, 0xa4, 0x80, 0x65, 0x63, 0xa8, 0x8d, 0x53, 0x83, 0x9a, 0x78, 0x77, 0x8a, 0x7d, 0x6c, 0x91, 0x81, 0x7a, 0x80, 0x75, 0x87, 0x94, 0x64, 0x7e, 0x97, 0x6f, 0x7a, 0x8c, 0x7c, 0x7d, 0x83, 0x77, 0x83, 0x82, 0x8a, 0x8c, 0x47, 0xad, 0xa1, 0x30, 0x97, 0x94, 0x5d, 0x94, 0x8c, 0x60, 0x80, 0xac, 0x5c, 0x74, 0xab, 0x5d, 0x86, 0x9a, 0x5a, 0x89, 0x8a, 0x7e, 0x70, 0x8b, 0x68, 0x9c, 0x9a, 0x3c, 0x83, 0xcb, 0x47, 0x63, 0xab, 0x82, 0x6e, 0x80, 0x6a, 0xb2, 0x75, 0x56, 0xa2, 0x8a, 0x61, 0x74, 0xa3, 0x79, 0x6e, 0x79, 0x9f, 0x7d, 0x5a, 0x8e, 0x95, 0x74, 0x6c, 0x92, 0x84, 0x75, 0x81, 0x79, 0x83, 0x94, 0x6c, 0x75, 0x91, 0x97, 0x3b, 0xa2, 0xc2, 0x13, 0xb6, 0x90, 0x53, 0xae, 0x6b, 0x69, 0x9b, 0x86, 0x60, 0x7f, 0xa2, 0x4d, 0x87, 0xa9, 0x49, 0x94, 0x92, 0x73, 0x4e, 0x9e, 0xae, 0x3c, 0x8b, 0x9b, 0x8a, 0x4e};

ISR(TIMER1_OVF_vect)
{
	stopTimer();
	
	OCR1A = voice[pointer0];
	pointer0++;
	if (pointer0 >= simpeSize) {
		pointer0 = 0;
	}

	OCR1B = voice[pointer1];
	pointer1++;
	if (pointer1 >= simpeSize) {
		pointer1 = 0;
	}

	OCR2 = voice[pointer2];
	pointer2++;
	if (pointer2 >= simpeSize) {
		pointer2 = 0;
	}

	if (moutionSensorStop) {
		currentCicle++;
		if (currentCicle >= ciclesInSecond) {
			currentCicle = 0;

			seconds++;
			if (seconds >= sensorSecondsOfSuspension) {
				seconds = 0;
				moutionSensorStop = false;
			}
		}
	}
	
	startTimer();
};

ISR(ADC_vect)
{
	if (ADC >= 600 || moutionSensorStop) {
		PORTD |= (1<<0);
		
		if (!moutionSensorStop) {
			moutionSensorStop = true;
			seconds = 0;
			currentCicle = 0;
		}
	} else {
		PORTD &= ~(1<<0);
	}
};

int main(void)
{
	initTimerOne();
	initMotionSensor();
	sei();

    while (1) 
    {

    }
}

void initTimerOne()
{
	simpeSize = sizeof(voice)/sizeof(char);

	//Normal PWM non-inverting mode
	TCCR1A &= ~(1<<COM1A0);
	TCCR1A |= (1<<COM1A1);
	TCCR1A &= ~(1<<COM1B0);
	TCCR1A |= (1<<COM1B1);

	//Fast PWM 8-bit
	TCCR1A |= (1<<WGM10);
	TCCR1A &= ~(1<<WGM11);
	TCCR1B |= (1<<WGM12);
	TCCR1B &= ~(1<<WGM13);

	//Timer/Counter 1 overflow interrupt settings
	//Interrupt enable
	TIMSK |= (1<<TOIE1);

	//Fast PWM timer 2
	TCCR2 |= (1<<WGM21) | (1<<WGM20);
	//Normal PWM non-inverting mode
	TCCR2 &= ~(1<<COM20);
	TCCR2 |= (1<<COM21);

	//Timer/Counter 2 start 1/8
	TCCR2 &= ~(1<<CS20);
	TCCR2 |= (1<<CS21);
	TCCR2 &= ~(1<<CS22);

	OCR1A = 0;
	OCR1B = 0;
	OCR2 = 0;

	startTimer();
}

void initMotionSensor()
{

	DDRD |= (1<<0);
	PORTD &= ~(1<<0);

	DDRC &= ~(1<<0);

	//ADC configuration
	//ADC enable
	ADCSRA |= (1<<ADEN);
	//непрерывное преобразование
	ADCSRA |= (1<<ADFR);

	//125 kHz
	ADCSRA |= (1<<ADPS1) | (1<<ADPS0);
	ADCSRA &= ~(1<<ADPS2);
	//Interrupt enable
	ADCSRA |= (1<<ADIE);

	//2,56v voltage reference
	ADMUX |= (1<<REFS1) | (1<<REFS0);
	//Right side
	ADMUX &= ~(1<<ADLAR);

	//ADC start
	ADCSRA |= (1<<ADSC);
}

void startTimer() {
	//Timer/Counter 1 start 1/8
	TCCR1B &= ~(1<<CS10);
	TCCR1B |= (1<<CS11);
	TCCR1B &= ~(1<<CS12);

	TCNT1 = 65530;
}

void stopTimer() {
	//Timer/Counter 1 stop
	TCCR1B &= ~(1<<CS10);
	TCCR1B &= ~(1<<CS11);
	TCCR1B &= ~(1<<CS12);
}

