#pragma once
#include <stdint.h>

/* NonvolatilePreference is a library for conveniently storing variables in
   EEPROM, so they can be used as preferences between program restarts. */
#include <NonvolatilePreference.hpp> // https://github.com/VaSe7u/NonvolatilePreference

// Time is in microseconds


namespace config {

NonvolatilePreferences::setStartingAddress(5);
NonvolatilePreferences::restoreDefaultsOnce(1);
/* The startning address of 'NVP' variables is set to 5.
   If the argument of 'restoreDefaultsOnce' is changed
   the 'NVP' variables will be resotred to their defaults. */

namespace debug {
const auto baud = 2000000;
#define DEBUGGING true // Enables UART debugging message.
#define VERBOSE true // Also prints line and function.

#define SYNCHRONIZE true
/* Disable to measure the cycle period. */

// These will be printed even if 'DEBUGGING' is disabled.
#define DEBUG_CYCLE_PERIOD true // Print cycle period.
#define DEBUG_SYNC_PERIOD true // Print sync block period.

#define DEBUG_ANGULAR_VELOCITY false // in deg/s
#define DEBUG_ACCELERATION false // raw
#define DEBUG_MAGNETIC_FIELD false // raw
#define DEBUG_PRESSURE false // in hPa
} //namespace debug


const int8_t cycles = 2; // 3 (including zero)
/* The program is divided in cycles. The main part of the
   program is executed every cycle (reading IMU, fusing,
   calculating controllers...). The less important (or less
   frequently changing) parts of the program are executed
   only in their own cycle. */
/* Cycles:
     0 - outer controller
     1 - communication
     2 - indication or battery
*/
const int8_t lowPriorityCycle = 2;
/* The low priority cycle consists of subcycles which execute
   tasks one at a time. These are the least important parts of the
   program (e.g. updating indication, reading battery...). */

constexpr uint16_t cyclePeriod = 2500;
/* This is the maximum time (in microseconds) it takes for a cycle
   to complete, in other words this is the slowest cycle in the
   worst case scenario. To determine the slowest cycle period,
   DEBUG_CYCLE_PERIOD has to be enabled and SYNCHRONIZE has to be
   disabled. */


namespace imu {
namespace mpu6050 {
// if using mpu6050
} // namespace mpu6050
namespace mpu925x { // link to API
namespace accelerometer {
const int8_t range = 1; // 2g
const int8_t dlpf = 2; // 2.88ms delay, 99Hz bandwidth
} //namespace accelerometer
namespace gyroscope {
const int8_t range = 1; // 500deg/s
const int8_t dlpf = 1; // 2.9ms delay, 184Hz bandwidth
} //namespace gyroscope
} //namespace mpu925x
const bool deriveFromFused = true;
/* When 'deriveFromFused' is true the angular velocities
   are calculated from the fused angle, if it is false
   they are taken directly from the gyroscope. The former
   should method should be less noisy. */
} //namespace imu

namespace fusion {
namespace mahony {
const float p = 0.5f;
const float i = 0.05f;
/* Higher Mahony gains favor the accelerometer angle more.
   This can lead to more accurate but slower results. */
} //namespace mahony
} //namespace fusion

namespace altimeter {
namespace bmp280 {
const int8_t pressureOversampleRatio = 16;
const int8_t temperatureOversampleRatio = 2;
const int8_t filterRatio = 16;
const int8_t standbyTime = 0;
} //namespace bmp280
} //namespace altimeter

namespace controller {
// try using PIDconfig object here
namespace inner {
NVP<float> p(0.2f); // https://en.wikipedia.org/wiki/PID_controller
const uint16_t updatePeriod = cyclePeriod; // Executed every cycle.
const int16_t outputRange = 127;
/* The output range for the three inner controllers is -127..127.
   Using a high output range with a proportional only controller is
   not a problem. */
namespace yaw {
NVP<float> p(0.4f);
/* The yaw controller needs a controller with higher proportional
   gain, because it uses different (less powerful) technique for
   creating yaw movement (gyroscopic effect). */
NVP<float> i(0.1f);
/* The integral term is used to eliminate steady-state error.
   In this case the small rotaton of the drone is not registered
   because it is rounded to zero, resulting in constant drift. */
const int16_t maximalIntegralSum = 60;
/* The accumulation of integral error is capped at 60. */
} //namespace yaw
} //namespace inner
namespace outer {
NVP<float> p(2.0f);
/* The proportional term of the outer controllers is multiplied
   by the current angle error. Increasing the proportional term
   too much will make the drone oscillate quickly and
   generally more unstable. */
NVP<float> i(0.5f);
/* The integral term in the outer controllers will make the drone
   more stable and will minimize it's pitch and roll drifting. Too
   much integral term will make it oscillate slowly. */
const uint16_t updatePeriod = cyclePeriod * (cycles + 1); //check if accurate
const int16_t outputRange = 180;
const int16_t maximalIntegralSum = 60;
} //namespace outer
} //namespace controller

namespace communication {
namespace nrf24l01p { // http://tmrh20.github.io/RF24/index.html
const int8_t cycle = 1;
// The cycle that will execute the communications part of the program.
const uint16_t timeout = 350000;
/* The timeout (in microseconds) for loss of communication. If
   no 'command' message was received for that time, the drone
   will enter 'FlyMode::noCommunication' state. */
NVP<int8_t> telemetryType(0);
/* A 'telemetry' message is returned as acknowledgment every time
   a 'command' message is received. The telemetry type specifies
   what information will be returned (e.g. battery voltage, altitude,
   IMU's output, controller's output...). */
const int8_t paLevel = 3; // RF24_PA_MAX
const int8_t dataRate = 1; // RF24_2MBPS
const int8_t retriesCount = 1; // One retry,
const int8_t retriesDelay = 0; // with zero delay
const int8_t crcLength = 2; // RF24_CRC_16
// const uint64_t writingPipe = 0x3A3A3A3AD2LL;
// const uint64_t readingPipe = 0x3A3A3A3AC3LL;
// uint8_t writingPipe[] = {0xD2, 0x3A, 0x3A, 0x3A, 0x3A};
// uint8_t readingPipe[] = {0xC3, 0x3A, 0x3A, 0x3A, 0x3A};
const int8_t addressWidth = 3;
const uint8_t pipe = 0x3A;
const uint8_t remotePipe = 0xD2;
const uint8_t dronePipe = 0xC3;
/* Every nRF24L01+ can have 6 receiving addresses (pipes) and
   1 sending address. In this case the remote is the sending
   address and the drone is receiving one. The address consists
   of 3 to 5 bytes, the first byte has to be different from the
   others. */
} //namespace nrf24l01p
} //namespace communication

namespace indication {
const uint16_t period = 250000; // The blink period of the LEDs
NVP<uint8_t> armsLevel(16);
/* The arms level is specified in 32 steps. The step is converted
   to actual PWM from a look-up array. This is done because linear
   change in brighness is not perceived as such.
   https://github.com/VaSe7u/ExponentMap */
NVP<bool> lampState(false);
} //namespace indication

namespace battery {
const float lpfAlpha = 0.9; // Low-pass filter
const float lowMargin = 11.1; // Vb < 11.1V is considered low
const float criticalMargin = 10.3; // Vb < 10.3V is considered critical
} //namespace battery

} //namespace config
