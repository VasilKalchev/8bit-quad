#ifndef _CONFIG_HPP_
#define _CONFIG_HPP_

namespace config {

	#define DEBUG_CYCLE_PERIOD (0)

	#define TIME_REMOTE_CONTROL (0)

	#define PRINT_REMOTE_CONTROL (0)
	#define PRINT_IMU (0)
	#define PRINT_SENSOR_FUSION (0)
	#define PRINT_CONTROLLER_OUTER (0)
	#define PRINT_CONTROLLER_INNER (0)
	#define PRINT_THROTTLE_COMPENSATION (0)
	#define PRINT_MOTOR_MIX (0)
	#define PRINT_BLACK_BOX (0)
	#define PRINT_PROCESS_CONFIG (0)
	#define PRINT_UPDATE_INDICATION (0)
	#define PRINT_READ_BATTERY (0)

} // namespace config

#endif // _CONFIG_HPP_


#pragma once
#include <inttypes.h>
#include <stdlib.h>

#include "EepromSetting.hpp"
#include "Communication.hpp"


namespace config {

const uint16_t cycleTime = 3100;

namespace debug {

#define DEBUG_PIN true
#define DBG_PIN_LOOP_PERIOD false
#define DBG_PIN_INNER_PERIOD false
#define DBG_PIN_OUTER_PERIOD false
#define DBG_PIN_MAHONY_PERIOD true

#define DEBUGGING true
extern const uint32_t baud;

#define DEBUG_SETTINGS false

#define DEBUG_TAKE_OFF_PRESSURE true
#define DEBUG_PRESSURE false
#define DEBUG_ALTITUDE true
#define DEBUG_VERTICAL_SPEED true
#define DEBUG_VERT_ACCEL_SETPOINT true

#define DEBUG_LOOP_TIME false

#define DEBUG_ANGULAR_VELOCITY false
#define DEBUG_ACCELERATION false
#define DEBUG_ACCELEROMETER_ANGLE false
#define DEBUG_ATTITUDE false

#define DEBUG_PID_PITCH false
#define DEBUG_PID_ROLL false
#define DEBUG_PID_YAW false
#define DEBUG_MOTORS false

#define DEBUG_COMMAND false
} //namespace debug


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

namespace regulation {
namespace inner {
extern EepromSetting<float> P;
extern EepromSetting<float> yawP;
extern const float yawI;
extern const uint32_t yaw_updateRate;
extern const int16_t outputLimit;
// extern EepromSetting<int16_t> outputLimit;
} //namespace inner
namespace outer {
extern EepromSetting<float> P;
extern EepromSetting<float> I;
extern const uint32_t updateRate;
// extern EepromSetting<uint32_t> updateRate;
extern const int16_t outputLimit;
// extern EepromSetting<int16_t> outputLimit;
// extern const uint32_t updateRateTolerance;
} //namespace outer
extern const uint8_t minimumRegulationThrottle;
extern const uint8_t maximumBaseThrottle;
// extern EepromSetting<uint8_t> minimumRegulationThrottle;
// extern EepromSetting<uint8_t> maximumBaseThrottle;
namespace altitude {
extern EepromSetting<float> P;
extern EepromSetting<float> D;
} //namespace altitude
} //namespace regulation

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
