#pragma once
#include <inttypes.h>

#include <avr/io.h>
#include <Arduino.h>

#include "ADC.hpp"


namespace pin {

namespace motor {
namespace ino {
const uint8_t tl = 5;
const uint8_t tr = 10;
const uint8_t bl = 6;
const uint8_t br = 9;
} //namespace ino
const uint8_t tl = PD5;
const uint8_t tr = PB2;
const uint8_t bl = PD6;
const uint8_t br = PB1;
} //namespace motor

namespace indication {
namespace ino {
const uint8_t arms = 3;
const uint8_t lamp = 4;
const uint8_t signal = A2;
const uint8_t warning = A1;
} //namespace ino
const uint8_t arms = PD3;
const uint8_t lamp = PD4;
const uint8_t signal = PC2;
const uint8_t warning = PC1;
} //namespace indication

namespace input {
namespace ino {
const uint8_t batteryVoltage = A3;
} //namespace ino
const uint8_t batteryVoltage = PC3;
} //namespace input

namespace communication {
namespace ino {
const uint8_t ce = 7;
const uint8_t csn = 8;
const uint8_t interrupt = A0;
} //namespace ino
const uint8_t ce = PD7;
const uint8_t csn = PB0;
const uint8_t interrupt = PC0;
}

namespace imu {
namespace ino {
const uint8_t interrupt = 2;
} //namespace ino
const uint8_t interrupt = PD2;
} //namespace imu

} //namespace pin

/*
Not sure:
Ya is positive when back
Xa is positive when left
Za is positive when normal

Yg is positive when fronting
Xg is positive when righting
Zg is positive CCW
*/


void initIO();
void replaceTimer0WithTimer2();

void tl(uint8_t pwm);
void tr(uint8_t pwm);
void bl(uint8_t pwm);
void br(uint8_t pwm);

namespace indication {
void arms(uint8_t pwm);
uint8_t arms();
void toggleArms();

void lamp(bool state);
bool lamp();
void toggleLamp();

void signal(bool state);
bool signal();
void toggleSignal();

void warning(bool state);
bool warning();
void toggleWarning();

} //namespace indication

float readBatteryVoltage();