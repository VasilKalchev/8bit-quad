#pragma once
#include <inttypes.h>
#include <stdlib.h>

#include "EepromSetting.hpp"
#include "Communication.hpp"


namespace config {

const uint16_t cycleTime = 3100;


// TODO: check if defines are correct
#define FUSION_COMPLEMENTARY_FILTER false
#define FUSION_MAHONY_FILTER true
#define ANGULAR_RATE_DIRECT_FROM_GYROSCOPE false
#define ANGULAR_RATE_DIFFERENTIATE_FROM_MAHONY true


#define EE_FIRST_FREE_ADDRESS (200)

namespace debug {

#define DEBUG_PIN false
#define DBG_PIN_LOOP_PERIOD false
#define DBG_PIN_INNER_PERIOD false
#define DBG_PIN_OUTER_PERIOD false
#define DBG_PIN_MAHONY_PERIOD false

#define DEBUGGING false
extern const uint32_t baud;

#define DEBUG_SETTINGS false
#define DEBUG_BLACK_BOX_HOVER_VOLTAGE false
#define DEBUG_EEPROM false

#define DEBUG_TAKE_OFF_PRESSURE false
#define DEBUG_PRESSURE false
#define DEBUG_ALTITUDE false
#define DEBUG_VERTICAL_SPEED false
#define DEBUG_VERT_SPEED_CTRL false


#define DEBUG_COMMAND false
} //namespace debug

// Debugging
#define PRINT_LOOP_PERIOD false
#define DEBUG_LOOP_PERIOD false

#define DEBUG_RATE_CTRL_PERIOD false

#define DEBUG_ANGLE_CTRL_PERIOD false

#define PRINT_CTRL_PITCH false
#define PRINT_CTRL_ROLL false
#define PRINT_CTRL_YAW false

#define PRINT_ANGULAR_RATE false
#define PRINT_ACCELERATION false

#define PRINT_ATTITUDE_ACCEL false
#define PRINT_ATTITUDE false

#define PRINT_THROTTLE false
#define PRINT_MOTOR_SIGNALS false

#define PRINT_BATTERY_VOLTAGE false

#define PRINT_VERTICAL_SPEED false
// ~Debugging

#define BLACK_BOX_HOVER_VOLTAGE true
#define VERT_SPEED_HOVER_THRESHOLD 500

namespace imu {
// #define IMU_MPU6050
#define IMU_MPU925x

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
namespace elev {
extern EepromSetting<float> P;
extern EepromSetting<float> D;
} //namespace elev
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
