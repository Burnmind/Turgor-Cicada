/*
 * Cicada.c
 *
 * Created: 25.08.2019 12:45:42
 * Author : Burnmind
 */ 

#define F_CPU 1000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

#define ciclesInSecond 200
#define sensorSecondsOfSuspension 30
#define WAVE_SHIFT M_PI/100
#define PULSE_SHIFT M_PI/100000
#define VOLUME_SHIFT M_PI/500000

#define false 0
#define true !false

void startTimer();
void stopTimer();
void initTimerOne();
void initMotionSensor();
void processSound();
void processMotionSensor();
int transpose(int value, int minIn, int maxIn, int minOut, int maxOut);

int seconds = 0;
int currentCicle = 0;

int currentLvlF = 0;
int currentLvlS = 0;
int currentLvlT = 0;

float wavePosition = 0.0;

float sinWaveF = 0.0;
float sinWaveS = 0.0;
float sinWaveT = 0.0;

float pulsePositionF = 0.0;
float pulsePositionS = M_PI/2;
float pulsePositionT = M_PI/3;

float volumePositionF = 0.0;
float volumePositionS = M_PI/2;
float volumePositionT = M_PI/3;

char moutionSensorStop = false;

ISR(TIMER1_OVF_vect)
{
	stopTimer();

	processSound();

	processMotionSensor();
	
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

void processSound() {
	sinWaveF = sin(wavePosition)*sin(pulsePositionF)*sin(volumePositionF)*100;

	sinWaveS = sin(wavePosition)*sin(pulsePositionS)*sin(volumePositionS)*100;

	sinWaveT = sin(wavePosition)*sin(pulsePositionT)*sin(volumePositionT)*100;

	wavePosition += WAVE_SHIFT;

	pulsePositionF += PULSE_SHIFT;
	pulsePositionS += PULSE_SHIFT;
	pulsePositionT += PULSE_SHIFT;
	
	volumePositionF += VOLUME_SHIFT;
	volumePositionS += VOLUME_SHIFT;
	volumePositionT += VOLUME_SHIFT;

	if (M_PI*2 < wavePosition) {
		wavePosition = 0;
	}

	if (M_PI < pulsePositionF) {
		pulsePositionF = 0;
	}
		
	if (M_PI < pulsePositionS) {
		pulsePositionS = 0;
	}
		
	if (M_PI < pulsePositionT) {
		pulsePositionT = 0;
	}

	if (M_PI*2 < volumePositionF) {
		volumePositionF = 0;
	}
	
	if (M_PI*2 < volumePositionS) {
		volumePositionS = 0;
	}
	
	if (M_PI*2 < volumePositionT) {
		volumePositionT = 0;
	}

	currentLvlF = transpose((int)sinWaveF, -100, 100, 0, 255);
		
	OCR1A = currentLvlF;

	currentLvlS = transpose((int)sinWaveS, -100, 100, 0, 255);

	OCR1B = currentLvlS;

	currentLvlT = transpose((int)sinWaveT, -100, 100, 0, 255);
		
	OCR2 = currentLvlT;
}

void processMotionSensor() {
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
}

int transpose(int value, int minIn, int maxIn, int minOut, int maxOut) {
	return (maxOut*(minIn - value) - minOut*(value - maxIn)) / (minIn - maxIn);
}

