/*
 * Cicada.c
 *
 * Created: 25.08.2019 12:45:42
 * Author : Burnmind
 */ 

#include <avr/io.h>


int main(void)
{
	//Normal PWM non-inverting mode
	TCCR1A &= ~(1<<COM1A0);
	TCCR1A |= (1<<COM1A1);

	//Fast PWM 10-bit
	TCCR1A |= (1<<WGM10);
	TCCR1A |= (1<<WGM11);
	TCCR1B |= (1<<WGM12);
	TCCR1B &= ~(1<<WGM13);

	//Divisor - 1 (No prescaling)
	TCCR1B &= ~(1<<CS10);
	TCCR1B |= (1<<CS11);
	TCCR1B &= ~(1<<CS12);


	// PB1 - output
	DDRB |= (1<<1);
	PORTB &= ~(1<<1);

	OCR1A = 512;

	//Power managment - Open
	DDRD |= (1<<0);
	PORTD |= (1<<0);

    while (1) 
    {
    }
}

