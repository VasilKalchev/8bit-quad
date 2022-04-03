// Constant speed

// Acceleration, slow

// Acceleration, fast


const uint32_t period = 500;

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
    Serial.begin(2000000);
    hw_init();

    Serial.print("RESET ---\n\n-----Motor test-----\n");
}

void loop() {
    static int8_t motor = 0;
    static uint8_t speed = 0;
    static int8_t mode = -1;

    if (Serial.available() > 0) {
        String rec = Serial.readString();
        if (rec.equalsIgnoreCase(String("tl"))) {
            motor = 0;
            Serial.println("selected TL");
        } else if (rec.equalsIgnoreCase(String("tr"))) {
            motor = 1;
            Serial.println("selected TR");
        } else if (rec.equalsIgnoreCase(String("bl"))) {
            motor = 2;
            Serial.println("selected BL");
        } else if (rec.equalsIgnoreCase(String("br"))) {
            motor = 3;
            Serial.println("selected BR");
        } else if (rec.equalsIgnoreCase(String("acc"))) {
            mode = 0;
            Serial.println("acceleration mode");
        } else if (rec.equalsIgnoreCase(String("c"))) {
            mode = 1;
            Serial.println("constant speed mode");
        } else if (rec.equalsIgnoreCase(String("s"))) {
            mode = -1;
            Serial.println("stop");
        } else {
            speed = rec.toInt();
            Serial.print("speed: "); Serial.println(speed);
        }
    }

    if (mode == 0) {
        static uint32_t lm = millis();
        if (millis() - lm > period) {
            lm = millis();
            if (speed < 127) {
                ++speed;
            } else {
                speed = 0;
            }

            static bool ledState = true;
            signal(ledState);
            ledState = !ledState;
        }
    }


    if (mode == 1 || mode == 0) {
        switch (motor) {
            case 0:
            motor_top_left(speed + 127);
            warning(false);
            break;
            case 1:
            motor_top_right(speed + 127);
            warning(false);
            break;
            case 2:
            motor_bottom_left(speed + 127);
            warning(false);
            break;
            case 3:
            motor_bottom_right(speed + 127);
            warning(false);
            break;
            default:
            warning(true);
            break;
        }
    } else {
        motor_top_left(127);
        motor_top_right(127);
        motor_bottom_left(127);
        motor_bottom_right(127);

        signal(false);
        warning(true);
    }
}
