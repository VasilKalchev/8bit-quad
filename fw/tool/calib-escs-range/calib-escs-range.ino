uint16_t const high_delay = 1000;
uint16_t const inter_esc_delay = 2000;
uint8_t const pwm_high = 254;
uint8_t const pwm_low = 127;

namespace pin {
namespace indication {
const uint8_t arms = PD3;
const uint8_t lamp = PD4;
const uint8_t signal = PC2;
const uint8_t warning = PC1;
} // namespace indication
} // namespace pin


void hw_init() {
  // // Configure Timer 2 to replace Timer 0.
  TIMSK0 &= ~(1 << TOIE0); // disable overflow interrupt for timer0
  // TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); //outputs, fast PWM with TOP = 0xff
  // TCCR2B = _BV(CS22);   // clock at F_CPU / 64
  // TIMSK2 |= (1 << TOIE2); // enable overflow interrupt for timer2

  // Set motors PWM to 0
  OCR0B = 0;
  OCR1B = 0;
  OCR0A = 0;
  OCR1A = 0;

  // Motors
  DDRD |= (1 << PD5) | (1 << PD6);
  DDRB |= (1 << PB2) | (1 << PB1);

  // Indication
  DDRC |= (1 << pin::indication::signal) | (1 << pin::indication::warning);
  DDRD |= (1 << pin::indication::arms) | (1 << pin::indication::lamp);

  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00); // Timer0 phase correct PWM
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | _BV(WGM10); // Timer1 phase correct PWM
}


void motor_top_left(uint8_t pwm) {
  OCR0B = pwm;
}

void motor_top_right(uint8_t pwm) {
  OCR1B = pwm;
}

void motor_bottom_left(uint8_t pwm) {
  OCR0A = pwm;
}

void motor_bottom_right(uint8_t pwm) {
  OCR1A = pwm;
}

void signal(bool state) {
  if (state) PORTC |= (1 << pin::indication::signal);
  else PORTC &= ~(1 << pin::indication::signal);
}

void warning(bool state) {
  DDRC |= (1 << PC1);
  if (state) PORTC |= (1 << pin::indication::warning);
  else PORTC &= ~(1 << pin::indication::warning);
}


void setup() {
  hw_init();
  signal(false);
  warning(true);

  Serial.begin(2000000);

  Serial.print("RESET ---\n\n-----ESC calibration-----\n");
  Serial.print("Starting ESC calibration...\n");

  // Top left
  Serial.print("TOP LEFT:\n");
  Serial.print(" high (254) for "); Serial.print(high_delay);
  Serial.print("milliseconds\n");
  motor_top_left(pwm_high);
  signal(false);
  delay(high_delay);

  Serial.print(" low (127)\n");
  motor_top_left(pwm_low);
  signal(true);

  Serial.print("waiting "); Serial.print(inter_esc_delay);
  Serial.print(" milliseconds\n\n");
  delay(inter_esc_delay);
  // ------

  // Top right
  Serial.print("TOP RIGHT:\n");
  Serial.print(" high (254) for "); Serial.print(high_delay);
  Serial.print("milliseconds\n");
  motor_top_right(pwm_high);
  signal(false);
  delay(high_delay);

  Serial.print(" low (127)\n");
  motor_top_right(pwm_low);
  signal(true);

  Serial.print("waiting "); Serial.print(inter_esc_delay);
  Serial.print(" milliseconds\n\n");
  delay(inter_esc_delay);
  // ------

  // Bottom left
  Serial.print("BOTTOM LEFT:\n");
  Serial.print(" high (254) for "); Serial.print(high_delay);
  Serial.print("milliseconds\n");
  motor_bottom_left(pwm_high);
  signal(false);
  delay(high_delay);

  Serial.print(" low (127)\n");
  motor_bottom_left(pwm_low);
  signal(true);

  Serial.print("waiting "); Serial.print(inter_esc_delay);
  Serial.print(" milliseconds\n\n");
  delay(inter_esc_delay);
  // ------

  // Bottom right
  Serial.print("BOTTOM RIGHT:\n");
  Serial.print(" high (254) for 1 second\n");
  motor_bottom_right(pwm_high);
  signal(false);
  delay(high_delay);

  Serial.print(" low (127)\n");
  motor_bottom_right(pwm_low);
  signal(true);
  // ------

  Serial.print("\nDone!\n");
  signal(true);
  warning(true);
}

void loop() {}
