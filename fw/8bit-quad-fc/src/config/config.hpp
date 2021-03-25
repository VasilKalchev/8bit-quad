#pragma once
#include <inttypes.h>
#include <stdlib.h>

#include "EepromSetting.hpp"
#include "../common/comm_packets.hpp"


namespace config {

const uint16_t cycleTime = 3100;

// Marks the first EEPROM address not used by any EepromSetting above -
// free for future black-box/logging use.
#define EE_FIRST_FREE_ADDRESS (200)

namespace debug {

#define DEBUGGING false
extern const uint32_t baud;

#define DEBUG_SETTINGS true

#define DEBUG_LOOP_TIME false

#define DEBUG_ANGULAR_VELOCITY false
#define DEBUG_ACCELERATION false
#define DEBUG_ATTITUDE false

#define DEBUG_PID_PITCH false
#define DEBUG_PID_ROLL false
#define DEBUG_PID_YAW false
#define DEBUG_MOTORS false

#define DEBUG_COMMAND false
} //namespace debug


namespace imu {
extern const uint8_t i2cAddress;
namespace lowPassFilter {
// extern EepromSetting<int8_t> common;
// namespace angularVelocity {
// extern EepromSetting<bool> state;
// extern EepromSetting<float> alpha;
// extern float oneMinusAlpha;
// } //namespace angularVelocity
// namespace acceleration {
// extern EepromSetting<bool> state;
// extern EepromSetting<float> alpha;
// extern float oneMinusAlpha;
// } //namespace acceleration
} //namespace lowPassFilter
// namespace complementary {
// extern EepromSetting<float> alpha;
// extern float oneMinusAlpha;
// } //namespace complementary
// extern const float epsilon;
} //namespace imu

namespace communication {
extern const uint32_t commandTimeout;
extern const int8_t powerAmplification;
extern const int8_t dataRate;
extern const int8_t retryDelay;
extern const int8_t retryCount;
extern const int8_t crcLength;
// extern EepromSetting<int8_t> powerAmplification;
// extern EepromSetting<int8_t> dataRate;
// extern EepromSetting<int8_t> retryDelay;
// extern EepromSetting<int8_t> retryCount;
// extern EepromSetting<int8_t> crcLength;
namespace telemetry {
extern EepromSetting<int8_t> type;
} //namespace telemetry
} //namespace communication

namespace ctlr {
namespace att {
namespace rate {
extern EepromSetting<float> P;
extern EepromSetting<float> yawP;
extern const float yawI;
extern const uint32_t yaw_updateRate;
extern const int16_t outputLimit;
// extern EepromSetting<int16_t> outputLimit;
} // namespace rate
namespace angle {
extern EepromSetting<float> P;
extern EepromSetting<float> I;
extern const uint32_t updateRate;
// extern EepromSetting<uint32_t> updateRate;
extern const int16_t outputLimit;
// extern EepromSetting<int16_t> outputLimit;
// extern const uint32_t updateRateTolerance;
} // namespace angle
} // namespace att
extern const uint8_t minimumRegulationThrottle;
extern const uint8_t maximumBaseThrottle;
// extern EepromSetting<uint8_t> minimumRegulationThrottle;
// extern EepromSetting<uint8_t> maximumBaseThrottle;
} //namespace ctlr

namespace indication {
extern const uint32_t period;
extern EepromSetting<uint8_t> armsLevel;
extern EepromSetting<bool> lamp;
} //namespace indication

namespace battery {
extern const uint32_t updateRate;
extern const float alpha;
extern const float lowVoltage;
extern const float criticalVoltage;
} //namespace battery

void init();
} //namespace config
