#include "mad0.hpp"


namespace board {

void ReplaceTimer0WithTimer2();
void SetupIO();
void SetupTimers();


void ReplaceTimer0WithTimer2() {
	// Configure Timer 2 to replace Timer 0.
	TIMSK0 &= ~(1 << TOIE0); // disable overflow interrupt for timer0
	TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); //outputs, fast PWM with TOP = 0xff
	TCCR2B = _BV(CS22);   // clock at F_CPU / 64
	TIMSK2 |= (1 << TOIE2); // enable overflow interrupt for timer2
}

void SetupIO() {
	// Set outputs
	DDRD |= (1 << pin::motor::tl) | (1 << pin::motor::bl);
	DDRB |= (1 << pin::motor::tr) | (1 << pin::motor::br);
	DDRC |= (1 << pin::indication::signal) | (1 << pin::indication::warning);
	DDRD |= (1 << pin::indication::arms) | (1 << pin::indication::lamp);
}

void SetupTimers() {
	TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00); // Timer0 phase correct PWM
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | _BV(WGM10); // Timer1 phase correct PWM
	// TCCR1B = _BV(CS12);
	DDRC |= (1 << PC1);
}

void Init() {
	ReplaceTimer0WithTimer2();
 	SetupIO();
 	SetupTimers();
}

void SetTL(uint8_t pwm) {
	OCR0B = pwm;
}

void SetTR(uint8_t pwm) {
	OCR1B = pwm;
}

void SetBL(uint8_t pwm) {
	OCR0A = pwm;
}

void SetBR(uint8_t pwm) {
	OCR1A = pwm;
}

} // namespace board
