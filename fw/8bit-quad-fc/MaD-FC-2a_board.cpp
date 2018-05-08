#include "mad-fc-2a_board.hpp"


// #define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
volatile uint32_t timer2OverflowCount = 0;

ISR(TIMER2_OVF_vect) { ++timer2OverflowCount; }

uint32_t us() {
  uint32_t m;
  uint8_t t;
  uint8_t oldSREG = SREG;
  cli();
  m = timer2OverflowCount;
  t = TCNT2;
  if (((TIFR2 & (1 << TOV2)) && (t < 255))) ++m;
  SREG = oldSREG;
  // return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
  return ((m << 8) + t) << 2;
}

// float ms() {
//   return ((float)us() / 1000.0f);
// }


namespace board {

void initializeIO() {
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
}

void initializeTimers() {
  disableTimer0OverflowInterrupt();
  configureTimer2ForUs();
  configureTimer0And1For490HzPhaseCorrectPwm();
}
void disableTimer0OverflowInterrupt() {
  TIMSK0 &= ~(1 << TOIE0); // disable overflow interrupt for timer0
}
void configureTimer2ForUs() {
  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); //outputs, fast PWM with TOP = 0xff
  TCCR2B = (1 << CS22); // clock at (F_CPU / 2^timerBits) / 64
  TIMSK2 |= (1 << TOIE2); // enable overflow interrupt for timer2
}
void configureTimer0And1For490HzPhaseCorrectPwm() {
  // Timer 0 and 1 phase correct PWM with TOP = 0xff
  TCCR0A = (1 << WGM00);
  TCCR1A = (1 << WGM10); // ... 8bit

  // Timer 0 and 1 compare output modes for A and B in phase correct PWM - clear then set
  TCCR0A |= (1 << COM0A1) | (1 << COM0B1);
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1);

  // Timer 0 and 1 prescaler 64: 490Hz.
  // ((F_CPU / 2^timerBits) / prescaler) / 2 = 490Hz
  TCCR0B |= (1 << CA01) | (1 << CS00);
  TCCR1B |= (1 << CS11) | (1 << CS10);
}


namespace motor {
void tl(const uint8_t pwm) {
  OCR0B = pwm;
}
void tr(const uint8_t pwm) {
  OCR1B = pwm;
}
void bl(const uint8_t pwm) {
  OCR0A = pwm;
}
void br(const uint8_t pwm) {
  OCR1A = pwm;
}
} //namespace motor

namespace indication {
const uint8_t armsLevel() {
  if (!(TCCR2A & (1 << COM2B1))) return (PORTD & (1 << PD3)) * 255;
  else return OCR2B;
}
void arms(const uint8_t pwm) {
  if (pwm == 0) {
    TCCR2A &= ~(1 << COM2B1);
    PORTD &= ~(1 << pin::indication::arms);
  } else if (pwm == 255) {
    TCCR2A &= ~(1 << COM2B1);
    PORTD |= (1 << pin::indication::arms);
  } else {
    TCCR2A |= (1 << COM2B1);
    OCR2B = pwm;
  }
}
void armsToggle() {
  TCCR2A &= ~(1 << COM2B1);
  PORTD ^= (1 << PD3);
}

const bool lampState() {
  return PORTD & (1 << pin::indication::lamp);
}
void lamp(const bool state) {
  if (state) PORTD |= (1 << pin::indication::lamp);
  else PORTD &= ~(1 << pin::indication::lamp);
}
void lampToggle() {
  PORTD ^= (1 << pin::indication::lamp);
}

const bool signalState() {
  return PORTC & (1 << pin::indication::signal);
}
void signal(const bool state) {
  if (state) PORTC |= (1 << pin::indication::signal);
  else PORTC &= ~(1 << pin::indication::signal);
}
void signalToggle() {
  PORTC ^= (1 << pin::indication::signal);
}

const bool warningState() {
  return PORTC & (1 << pin::indication::warning);
}
void warning(const bool state) {
  if (state) PORTC |= (1 << pin::indication::warning);
  else PORTC &= ~(1 << pin::indication::warning);
}
void warningToggle() {
  PORTC ^= (1 << pin::indication::warning);
}
} //namespace indication

} //namespace board
