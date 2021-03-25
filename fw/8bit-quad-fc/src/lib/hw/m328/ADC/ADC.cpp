#include "ADC.hpp"
#include <avr/io.h>
#include <util/delay.h>


void initADC() {


  // ADMUX |= (1 << REFS0); // AVCC reference voltage
  ADMUX |= (1 << REFS1) | (1 << REFS0); // internal 1.1V reference


  // ADCSR ADC Control and Status Register
  // ADCSRA |= (1 << ADPS2) | (1 << ADPS0); // ADC clock prescaler /32
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // clock prescaler /128

  PRR &= ~(1 << PRADC); // Disable ADC Power Reduction

  // ADEN ADC ENable
  ADCSRA |= (1 << ADEN); // enable ADC

  ADMUX = (0xf0 & ADMUX) | PC3;

  ADCSRA |= (1 << ADSC); // start first conversion
}

void initFreerunningADC() {
	ADCSRA |= (1 << ADATE); // enable auto-trigger
	initADC();
}

int16_t readADC(uint8_t channel) {
  ADMUX = (0b11110000 & ADMUX) | channel;
  // _delay_ms(1);
  ADCSRA |= (1 << ADSC);

  while (bit_is_set(ADCSRA, ADSC));
  return ADC;
}

int16_t readAnalogAverage(uint8_t pin, uint16_t readings, uint16_t us) {
  int32_t value = 0;
  for (uint16_t reading = 0; reading < readings; ++reading) {
    value += readADC(pin);
    _delay_us(us);
  }
  return value / readings;
}

  // if ((ADCSRA & (1 << ADSC)) == 0) {
  //  adcValue = ADC;
  //  ADCSRA |= (1 <<ADSC);
  //  return adcValue;
  // } else {
  //  ADCSRA |= (1 <<ADSC);
  //  return adcValue;
  // }

  // ADCSRA |= (1 << ADSC);                     /* start ADC conversion */

  // while ((ADCSRA & (1<<ADSC)) == 1) {}
  // // loop_until_bit_is_clear(ADCSRA, ADSC);          /* wait until done */
  // return ADC;                                     /* read ADC in */
  // // return (uint16_t)(ADCL | (ADCH << 8));