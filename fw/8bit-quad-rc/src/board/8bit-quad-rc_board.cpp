#include "8bit-quad-rc_board.hpp"


void replaceTimer0WithTimer2() {
  // Configure Timer 2 to replace Timer 0.
  TIMSK0 &= ~(1 << TOIE0); // disable overflow interrupt for timer0
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); //outputs, fast PWM with TOP = 0xff
  TCCR2B = _BV(CS22);   // clock at F_CPU / 64
  TIMSK2 |= (1 << TOIE2); // enable overflow interrupt for timer2
}

void initIO() {
  // Set inputs
  DDRB &= ~(1 << PB2);
  DDRC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC5));
  DDRD &= ~((1 << PD4) | (1 << PD2));

  // Set outputs
  DDRB |= (1 << PB1);
  DDRD |= (1 << PD3) | (1 << PD5) | (1 << PD6);

  // Set indication LEDs low
  PORTB &= ~(1 << PB1);
  PORTD &= ~((1 << PD3) | (1 << PD5) | (1 << PD6));

  // Enable joystick button pull-up resistor
  PORTB |= (1 << PB2);
  PORTC |= (1 << PC5);
  // Disable pull-ups on input pins
  PORTC &= ~(1 << PC3);
  PORTD &= ~((1 << PD2) | (1 << PD4));
}


int16_t readThrottle() {
  return readADC(pin::input::throttle);
}

int16_t readPitch() {
  return readADC(pin::input::pitch);
}

int16_t readRoll() {
  return readADC(pin::input::roll);
}

bool debounce(bool stateNow, float& state) {
  state = (state * 9.0f + stateNow * 10.0f) / 10.0f;
  return (state > 5.0f) ? true : false;
}

bool getFlightMode() {
  static float state = 0;
  return debounce(PINC & (1 << pin::input::flightMode), state);
}

bool getControlMode() {
  static float state = 0;
  state = (state * 9.0f + (PIND & (1 << pin::input::controlMode)) * 10.0f) / 10.0f;
  return (state > 5) ? true : false;
}

bool getYawMode() {
  // static float state = 0;
  // state = (state * 9.0f + (PIND & (1 << pin::input::yawMode)) * 10.0f) / 10.0f;
  // return (state > 5) ? true : false;
  return (PIND & (1 << pin::input::yawMode));
}

bool getFn() {
  static float state = 0;
  state = (state * 9.0f + (PINB & (1 << pin::input::fn)) * 10.0f) / 10.0f;
  return (state > 5) ? true : false;
}

bool getSw() {
  static float state = 0;
  state = (state * 9.0f + (PINC & (1 << pin::input::sw)) * 10.0f) / 10.0f;
  return (state > 5) ? true : false;
}


namespace indication {
namespace color {
void red(uint8_t level) { redLed(level); greenLed(0); blueLed(0); }
void green(uint8_t level) { redLed(0); greenLed(level); blueLed(0); }
void blue(uint8_t level) { redLed(0); greenLed(0); blueLed(level); }
void yellow(uint8_t level) { redLed(level / 2); greenLed(level / 2); blueLed(0); }
void cyan(uint8_t level) { redLed(0); greenLed(level / 2); blueLed(level / 2); }
void magenta(uint8_t level) { redLed(level / 2); greenLed(0); blueLed(level / 2); }
void white(uint8_t level) { redLed(level / 3); greenLed(level / 3); blueLed(level / 3); }
void black() { redLed(0); greenLed(0); blueLed(0); }
} //namespace color

void redLed(uint8_t level) {
  if (level == 0) {
    TCCR2A &= ~(1 << COM2B1);
    PORTD &= ~(1 << PD3);
  } else if (level == 255) {
    TCCR2A &= ~(1 << COM2B1);
    PORTD |= (1 << PD3);
  } else {
    TCCR2A |= (1 << COM2B1);
    OCR2B = level;
  }
}
void redLedToggle() {
  TCCR2A &= ~(1 << COM2B1);
  PORTD ^= (1 << PD3);
}

void greenLed(uint8_t level) {
  if (level == 0) {
    TCCR0A &= ~(1 << COM0B1);
    PORTD &= ~(1 << PD5);
  } else if (level == 255) {
    TCCR0A &= ~(1 << COM0B1);
    PORTD |= (1 << PD5);
  } else {
    TCCR0A |= (1 << COM0B1);
    OCR0B = level;
  }
}

void blueLed(uint8_t level) {
  if (level == 0) {
    TCCR0A &= ~(1 << COM0A1);
    PORTD &= ~(1 << PD6);
  } else if (level == 255) {
    TCCR0A &= ~(1 << COM0A1);
    PORTD |= (1 << PD6);
  } else {
    TCCR0A |= (1 << COM0A1);
    OCR0A = level;
  }
}

void warningLed(uint8_t level) {
  if (level == 0) {
    TCCR1A &= ~(1 << COM1A1);
    PORTB &= ~(1 << PB1);
  } else if (level == 255) {
    TCCR1A &= ~(1 << COM1A1);
    PORTB |= (1 << PB1);
  } else {
    TCCR1A |= (1 << COM1A1);
    OCR1A = level;
  }
}
void warningLedToggle() {
  TCCR1A &= ~(1 << COM1A1);
  PORTB ^= (1 << PB1);
}
} //namespace indication


// Timer output  Arduino output  Chip pin  Pin name
// OC0A  6 12  PD6
// OC0B  5 11  PD5
// OC1A  9 15  PB1
// OC1B  10  16  PB2
// OC2A  11  17  PB3
// OC2B  3 5 PD3
