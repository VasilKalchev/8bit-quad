#include "mad-fc-2_board.hpp"


void initIO() {
  // Set motors PWM to 0
  OCR0B = 0;
  OCR1B = 0;
  OCR0A = 0;
  OCR1A = 0;

  // Set inputs
  DDRC &= ~((1 << pin::communication::interrupt)
            | (1 << pin::input::batteryVoltage));
  DDRD &= ~(1 << pin::imu::interrupt);

  // Set outputs
  DDRD |= (1 << pin::motor::tl) | (1 << pin::motor::bl);
  DDRB |= (1 << pin::motor::tr) | (1 << pin::motor::br);
  DDRC |= (1 << pin::indication::signal) | (1 << pin::indication::warning);
  DDRD |= (1 << pin::indication::arms) | (1 << pin::indication::lamp);

  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00); // Timer0 phase correct PWM
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | _BV(WGM10); // Timer1 phase correct PWM
  // TCCR1B = _BV(CS12);
  DDRC |= (1 << PC1);
}

void replaceTimer0WithTimer2() {
  // Configure Timer 2 to replace Timer 0.
  TIMSK0 &= ~(1 << TOIE0); // disable overflow interrupt for timer0
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); //outputs, fast PWM with TOP = 0xff
  TCCR2B = _BV(CS22);   // clock at F_CPU / 64
  TIMSK2 |= (1 << TOIE2); // enable overflow interrupt for timer2
}

void tl(uint8_t pwm) {
  OCR0B = pwm;
}
void tr(uint8_t pwm) {
  OCR1B = pwm;
}
void bl(uint8_t pwm) {
  OCR0A = pwm;
}
void br(uint8_t pwm) {
  OCR1A = pwm;
}

namespace indication {
void arms(uint8_t pwm) {
  if (pwm == 0) {
    TCCR2A &= ~(1 << COM2B1);
    PORTD &= ~(1 << PD3);
  } else if (pwm == 255) {
    TCCR2A &= ~(1 << COM2B1);
    PORTD |= (1 << PD3);
  } else {
    TCCR2A |= (1 << COM2B1);
    OCR2B = pwm;
  }
}
uint8_t arms() {
  if (bit_is_clear(TCCR2A, COM2B1)) return (PORTD & (1 << PD3)) * 255;
  else return OCR2B;
}
void toggleArms() {
  TCCR2A &= ~(1 << COM2B1);
  PORTD ^= (1 << PD3);
}

void lamp(bool state) {
  DDRD |= (1 << pin::indication::lamp);
  if (state) PORTD |= (1 << pin::indication::lamp);
  else PORTD &= ~(1 << pin::indication::lamp);
}
bool lamp() {
  return PORTD & (1 << pin::indication::lamp);
}
void toggleLamp() {
  PORTD ^= (1 << pin::indication::lamp);
}

void signal(bool state) {
  if (state) PORTC |= (1 << pin::indication::signal);
  else PORTC &= ~(1 << pin::indication::signal);
}
bool signal() {
  return PORTC & (1 << pin::indication::signal);
}
void toggleSignal() {
  PORTC ^= (1 << pin::indication::signal);
}

void warning(bool state) {
  DDRC |= (1 << PC1);
  if (state) PORTC |= (1 << pin::indication::warning);
  else PORTC &= ~(1 << pin::indication::warning);
}
bool warning() {
  DDRC |= (1 << PC1);
  return PORTC & (1 << pin::indication::warning);
}
void toggleWarning() {
  DDRC |= (1 << PC1);
  PORTC ^= (1 << pin::indication::warning);
}

} //namespace indication

float readBatteryVoltage() {
  return (float)readADC(pin::input::batteryVoltage) * 0.01631671572899874532345445011472;
}