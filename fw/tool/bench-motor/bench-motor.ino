// tmr0
// tmr1 - pins 9 (OCR1A), 10 (OCR1B)
// tmr2

// Includes
#include <LiquidCrystal_I2C.h>


// Pin-out
const uint8_t pot_pin = A0;
const uint8_t current_pin = A1;
const uint8_t esc_voltage_pin = A2;
const uint8_t motor_pin = 5;
const uint8_t kill_sw_pin = 4;

// Config
const uint8_t lcd_address = 0x3F;
const uint8_t lcd_columns = 16;
const uint8_t lcd_rows = 2;

const float acs_offset = 2.5;
const float acs_sensitivity = 100;

const float adc2analog_multiplier = 5.0 / 1023.0;
const float esc_voltage_multiplier = 2.8;

const float out2ms = 8.034709946970914349991965290053;


// Global objects
LiquidCrystal_I2C lcd(lcd_address, lcd_columns, lcd_rows);



const uint32_t period = 500;


void hw_init() {
  // Configure Timer 2 to replace Timer 0.
  TIMSK0 &= ~(1 << TOIE0); // disable overflow interrupt for timer0

  // Set motor PWM to 0
  OCR0B = 0;

  // Motors
  DDRD |= (1 << PD5) | (1 << PD6);
  DDRB |= (1 << PB2) | (1 << PB1);

  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00); // Timer0 phase correct PWM
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | _BV(WGM10); // Timer1 phase correct PWM
}

void five(uint8_t pwm) {
  OCR0B = pwm;
}

void nine(uint8_t pwm) {
  OCR1A = pwm;
}


void setup() {
    Serial.begin(2000000);
    hw_init();

    lcd.init(); lcd.backlight();
    lcd.setCursor(3, 0); lcd.print("BLDC test");
    lcd.setCursor(5, 1); lcd.print("v1.0.0");

    pinMode(pot_pin, INPUT);
    pinMode(current_pin, INPUT);
    pinMode(kill_sw_pin, INPUT);

    Serial.print("RESET ---\n\n-----Motor test-----\n");

    while (digitalRead(kill_sw_pin) == 0);
}

void loop() {
    static uint16_t out = 0;

    // if (Serial.available() > 0) {
    //     String rec = Serial.readStringUntil('\n');
    //     if (rec.equalsIgnoreCase(String(""))) {
    //         five(127);
    //         nine(127);
    //         out = 0;
    //         Serial.println("emergency stop");
    //     } else {
    //         uint8_t speed = rec.toInt();
    //         out = speed;
    //         Serial.print("speed: "); Serial.println(speed);
    //     }
    // }

    // Read potentiometer
    uint16_t pot_raw = analogRead(pot_pin);
    float pot_v = (float)pot_raw * adc2analog_multiplier;

    // Convert pot to motor signal
    // 22 - 1014
    // 0 - 127
    if (pot_raw < 100) {
        out = 0;
    } else {
        out = map(pot_raw, 100, 1000, 0, 126);
    }

    if (digitalRead(kill_sw_pin) == 0) {
        five(127);
        nine(127);
        lcd.setCursor(2, 0); lcd.print("KILL SWITCH");
        lcd.setCursor(5, 1); lcd.print("IS ON");
        Serial.println("kill switch");
    } else {
        five(127 + out);
        nine(127 + out);
    }

    // Read ESC voltage
    uint16_t esc_v_raw = analogRead(esc_voltage_pin);
    float esc_v_div = esc_v_raw * adc2analog_multiplier;
    float esc_v = esc_v_div * esc_voltage_multiplier;

    // Read current
    // uint16_t current_raw = analogRead(current_pin);
    // float current_v = (float)current_raw * adc2analog_multiplier;
    // float current = ((float)current_v - acs_offset) / acs_sensitivity;

    // Display
    static uint32_t ms_last = 0;
    if (millis() - ms_last > 300) {
        ms_last = millis();

        static uint16_t out_last = 0;
        static uint16_t current_raw_last = 0.0;
        static uint16_t esc_v_raw_last = 0.0;

        // if ((out != out_last) || (current_raw != current_raw_last)) {
        if ((out != out_last) || (esc_v_raw != esc_v_raw_last)) {
            lcd.clear();
            lcd.setCursor(0, 0); lcd.print(out);
            lcd.setCursor(8, 0); lcd.print(1020 + (float)out * out2ms);
            lcd.setCursor(0, 1); lcd.print(esc_v); lcd.print(" V");
            // lcd.setCursor(8, 1); lcd.print(current); lcd.print(" A");

            // Serial.print("pot_raw: "); Serial.println(pot_raw);
            // Serial.print("pot_v: "); Serial.println(pot_v);
            Serial.print("out: "); Serial.println(out);
            Serial.print("tm: "); Serial.println(1020 + (float)out * out2ms);

            Serial.print("esc v: "); Serial.println(esc_v);

            // Serial.print("current_raw: "); Serial.println(current_raw);
            // Serial.print("current_v: "); Serial.println(current_v);
            // Serial.print("current: "); Serial.println(current);

            Serial.println();
        }

    }

    #if 0
    static uint8_t speed = 0;
    static int8_t mode = -1;

    if (Serial.available() > 0) {
        String rec = Serial.readStringUntil('\n');
        if (rec.equalsIgnoreCase(String(""))) {
            mode = -1;
            five(127);
            nine(127);
            Serial.println("emergency stop");
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
        }
    }


    if (mode == 1 || mode == 0) {
        five(speed + 127);
        nine(speed + 127);
    } else {
        five(127);
        nine(127);
    }
    #endif
}
