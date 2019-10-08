/*
 * Cicada.c
 *
 * Created: 25.08.2019 12:45:42
 * Author : Burnmind
 */ 

#define F_CPU 1000000UL
#include <avr/io.h>
#include <avr/interrupt.h>

void startTimer();
void stopTimer();

ISR(TIMER0_OVF_vect)
{
	stopTimer();
	//Do smth
	startTimer();
}

int main(void)
{
	//Normal PWM non-inverting mode
	TCCR1A &= ~(1<<COM1A0);
	TCCR1A |= (1<<COM1A1);

	//Fast PWM 8-bit
	TCCR1A |= (1<<WGM10);
	TCCR1A &= ~(1<<WGM11);
	TCCR1B |= (1<<WGM12);
	TCCR1B &= ~(1<<WGM13);

	startTimer();

	//Timer/Counter 1 overflow interrupt settings
	//Interrupt enable
	TIMSK |= (1<<2);

	// PB1 - output
	DDRB |= (1<<1);
	PORTB &= ~(1<<1);
	OCR1A = 0x00;

	//Power managment - Open
	DDRD |= (1<<0);
	PORTD |= (1<<0);

	sei();

    while (1) 
    {
    }
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

