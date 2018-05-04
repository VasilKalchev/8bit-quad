#pragma once
#include <inttypes.h>

#include <avr/io.h>
#include <Arduino.h>

#include "../peripheral/ADC.hpp"


namespace pin {

namespace input {
namespace ino {
const uint8_t throttle = A0;
const uint8_t pitch = A1;
const uint8_t roll = A2;
const uint8_t flightMode = A3;
const uint8_t controlMode = 4;
const uint8_t yawMode = 2;
const uint8_t fn = 10;
const uint8_t sw = A5;
} //namespace ino
const uint8_t throttle = PC0;
const uint8_t pitch = PC1;
const uint8_t roll = PC2;
const uint8_t flightMode = PC3;
const uint8_t controlMode = PD4;
const uint8_t yawMode = PD2;
const uint8_t fn = PB2;
const uint8_t sw = PC5;
} //namespace input

namespace indication {
namespace ino {
const uint8_t warning = 9;
const uint8_t red = 3;
const uint8_t green = 5;
const uint8_t blue = 6;
} //namespace ino
const uint8_t warning = PB1;
const uint8_t red = PD3;
const uint8_t green = PD5;
const uint8_t blue = PD6;
} //namespace indication

namespace communication {
namespace ino {
const uint8_t ce = 7;
const uint8_t csn = 8;
} //namespace ino
const uint8_t ce = PD7;
const uint8_t csn = PB0;
}

} //namespace pin


void replaceTimer0WithTimer2();

void initIO();

int16_t readThrottle();
int16_t readPitch();
int16_t readRoll();
bool getFlightMode();
bool getControlMode();
bool getYawMode();
bool getFn();
bool getSw();


namespace indication {
namespace color {
void red(uint8_t level);
void green(uint8_t level);
void blue(uint8_t level);
void yellow(uint8_t level);
void cyan(uint8_t level);
void magenta(uint8_t level);
void white(uint8_t level);
void black();
} //namespace color
void redLed(uint8_t level);
void redLedToggle();
void greenLed(uint8_t level);
void blueLed(uint8_t level);

void warningLed(uint8_t level);
void warningLedToggle();
} //namespace indication

/*
Throttle is positive when UP
Yaw is positive when RIGHT
Pitch is positive when UP
Roll is positive when RIGHT

Flight mode - 1 is ACRO
Control mode - 1 is MENU
Yaw mode - 0 is ENABLE
Function - 0 is ACTIVE
*/