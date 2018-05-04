#pragma once
#include <inttypes.h>


namespace config {

namespace debug {
#define DEBUGGING true
const uint32_t baud = 250000;

const bool telemetry = true;

#define DEBUG_LOOP_TIME false

#define DEBUG_CENTER_STICKS false
#define DEBUG_RAW_ANALOG_INPUTS false
#define DEBUG_DIGITAL_INPUTS false
#define DEBUG_COMMAND false

const bool batteryVoltage = true;
const bool altitude = true;
const bool telemetryRegulation = true;
const bool telemetryImu = true;
const bool telemetryMotors = true;
} //namespace debug


namespace adc {
const int8_t numOfReadings = 5;
const uint32_t delayBetweenReadings = 5;
} //namespace adc

namespace throttle {
const float lpfAlpha = 0.975;
const float lpfOneMinusAlpha = 1 - lpfAlpha;
} //namespace throttle

namespace stick {
const float lpfAlpha = 0.4;
const float lpfOneMinusAlpha = 1 - lpfAlpha;

const int16_t maxAngle = 35;
const int16_t maxAggressiveAngle = 45;
const int16_t maxAngularVelocity = 90;
const int16_t maxAggressiveAngularVelocity = 120;
const int16_t maxYawAngularVelocity = 60;
const int16_t maxYawAggressiveAngularVelocity = 90;
} //namespace stick

namespace communication {
const uint32_t commandPeriod = 50000;
} //namespace communication

namespace indication {
const uint32_t period = 150000;
} //namespace indication

namespace battery {
const bool indicateLowVoltage = true;
} //namespace battery

} //namespace config
