void setup() {
  // Configure Timer 2 to replace Timer 0.
  TIMSK0 &= ~(1 << TOIE0); // disable overflow interrupt for timer0
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); //outputs, fast PWM with TOP = 0xff
  TCCR2B = _BV(CS22);   // clock at F_CPU / 64
  TIMSK2 |= (1 << TOIE2); // enable overflow interrupt for timer2

  // Set outputs
  DDRD |= (1 << PD5) | (1 << PD6);
  DDRB |= (1 << PB2) | (1 << PB1);
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00); // Timer0 phase correct PWM
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | _BV(WGM10); // Timer1 phase correct PWM

  Serial.begin(2000000);

  Serial.println("254");
  OCR0B = 254;
  OCR1B = 254;
  OCR0A = 254;
  OCR1A = 254;
  delay(1000);
  OCR0B = 127;
  OCR1B = 127;
  OCR0A = 127;
  OCR1A = 127;
  Serial.println("127");
}

void loop() {}