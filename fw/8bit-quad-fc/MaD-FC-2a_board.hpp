#pragma once
#include <stdint.h>
#include <avr/io.h>

const uint32_t F_CPU = 16000000;

using namespace m328;

uint32_t us();
// float ms();


namespace board {

namespace pin {
namespace motor {
const uint8_t tl = PD5;
const uint8_t tr = PB2;
const uint8_t bl = PD6;
const uint8_t br = PB1;
} //namespace motor
namespace indication {
const uint8_t arms = PD3;
const uint8_t lamp = PD4;
const uint8_t signal = PC2;
const uint8_t warning = PC1;
} //namespace indication
namespace input {
namespace imu {
const uint8_t interrupt = PD2;
} //namespace imu
const uint8_t batteryVoltage = PC3;
} //namespace input
namespace communication {
const uint8_t ce = PD7;
const uint8_t csn = PB0;
const uint8_t interrupt = PC0;
} //namespace communication

// namespace ino {
// namespace motor {
// const uint8_t tl = 5;
// const uint8_t tr = 10;
// const uint8_t bl = 6;
// const uint8_t br = 9;
// } //namespace motor
// namespace indication {
// const uint8_t arms = 3;
// const uint8_t lamp = 4;
// const uint8_t signal = A2;
// const uint8_t warning = A1;
// } //namespace indication
// namespace input {
// namespace imu {
// const uint8_t interrupt = 2;
// } //namespace imu
// const uint8_t batteryVoltage = A3;
// } //namespace input
// namespace communication {
// const uint8_t ce = 7;
// const uint8_t csn = 8;
// const uint8_t interrupt = A0;
// } //namespace communication
// } //namespace ino

} //namespace pin


namespace imu {
namespace mpu925x {
namespace offset {
namespace accelerometer {
NVP<int16_t> x(2587);
NVP<int16_t> y(-2483);
NVP<int16_t> z(5226);
} //namespace accelerometer
namespace gyroscope {
NVP<int16_t> x(4);
NVP<int16_t> y(-6);
NVP<int16_t> z(2);
} //namespace gyroscope
} //namespace offset
} //namespace mpu925x
} //namespace imu


void initializeIO() __attribute__ ((naked)) __attribute__ ((section (".init3")));
void initializeTimers() __attribute__ ((naked)) __attribute__ ((section (".init3")));
void disableTimer0OverflowInterrupt() __attribute__ ((naked)) __attribute__ ((section (".init3")));
void configureTimer2ForUs() __attribute__ ((naked)) __attribute__ ((section (".init3")));
void configureTimer0And1For490HzPhaseCorrectPwm() __attribute__ ((naked)) __attribute__ ((section (".init3")));


namespace motor {
void tl(const uint8_t pwm);
void tr(const uint8_t pwm);
void bl(const uint8_t pwm);
void br(const uint8_t pwm);
} //namespace motor

namespace indication {
const uint8_t armsLevel();
void arms(const uint8_t pwm);
void armsToggle();

const bool lampState();
void lamp(const bool state);
void lampToggle();

const bool signalState();
void signal(const bool state);
void signalToggle();

const bool warningState();
void warning(const bool state);
void warningToggle();
} //namespace indication

const float analogReference = 1.1; // Measure it

namespace battery {
const float voltageDivider = 0.0654205607476635514018691588785; // R1 = 130kOhm, R2 = 9.1kOhm
// Battery voltage = analogRead() * (Analog Reference / (Analog Resolution * Voltage Divider))
constexpr float readingMultiplier = analogReference / (1024.0f * voltageDivider);
} //namespace battery

} //namespace board
