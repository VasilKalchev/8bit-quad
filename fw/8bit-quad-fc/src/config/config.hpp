#pragma once
#include <stdint.h>


/* Memory usage:
| Optimization | Flash | RAM  |
|--------------|-------|------|
| O0           | 96776 | 1632 |
| Os           | 19076 | 1067 |
| O1           | 20666 | 1041 |
| O2           | 20164 | 1081 |
| O3           | 27976 | 1081 |
| Ofast        | 27864 | 1081 |
*/

namespace cfg {

/* Attitude fusion method
 */
#define ATTITUDE_FUSION_METHOD_COMPLEMENTARY false
#define ATTITUDE_FUSION_METHOD_MAHONY true

#define COMPENSATE_THROTTLE_WHEN_TILTING (false)


uint32_t const uart_baud_rate = 2000000;

#define RESET_EEPROM_TO_DEFAULT true


namespace schedule {
uint16_t const sub_cycle_time = 2600;
uint16_t const cycle_time = sub_cycle_time * 4;

// angular rate ctrl: ~350usec
// read IMU:           500usec
// motor mix:          20usec
// ------------------- 870usec
// fusion (Mahony):    1000usec
// ------------------- 1730usec
enum class SubCycle_ndx : uint8_t {
  FIRST = 0,
  rc = 0, // 150usecs
  indication = 1, // 8usec
  battery = 1, // 160usec
  quat2euler = 2, // + 680usec
  angleCtrl = 3, // + ~240usec
  OUT_OF_RANGE,
};

/*
  | sub-cycle | sub-time | total time |
  |-----------|----------|------------|
  | 0 rc      | 80       | 1950       |
  | 1 b+ind   | 168      | 2040       |
  | 2 q2e     | 680      | 2550       |
  | 3 angle   | 240      | 2110       |

3-sub cycles variant
  - fuse every 6ms
  - fuse is outdated by 2ms
  | ndx | sub-cycle    | time | total time |
  |-----|--------------|------|------------|
  | 0   | fuse         | 1000 | 1940       |
  | 1   | q2e + angle  | 920  | 1860       |
  | 2   | aux          | 244  | 1090       |
 */
} // namespace schedule

namespace imu {
#define IMU_MPU6050
// #define IMU_MPU925x

constexpr uint32_t clock_speed = 800000;
constexpr uint8_t i2c_address = 0x68;  // const
extern bool median_filter_acc;

namespace offset {
extern int16_t accel[3];
extern int16_t gyro[3];
} // namespace offset
} // namespace imu


namespace eeprom {
namespace ctrlr {
namespace rate {
namespace tilt {
constexpr uint16_t p = 0x000;
constexpr uint16_t i = 0x004;
constexpr uint16_t d = 0x008;
} // namespace tilt
constexpr uint16_t p = 0x00C;
constexpr uint16_t i = 0x010;
constexpr uint16_t d = 0x014;
} // namespace rate
namespace angle {
constexpr uint16_t p = 0x018;
constexpr uint16_t i = 0x01C;
} // namespace angle
} // namespace ctrlr

namespace filter {
namespace mahony {
constexpr uint16_t p = 0x020;
constexpr uint16_t i = 0x024;
} // namespace mahony
namespace lpf {
constexpr uint16_t accel = 0x028;
constexpr uint16_t imu = 0x02C;
} // namespace lpf
} // namespace filter

namespace cfg {
constexpr uint16_t leds = 0x030;
constexpr uint16_t lamp = 0x031;
} // namespace cfg

namespace mpu6050 {
namespace accel {
constexpr uint16_t x = 0x040;
constexpr uint16_t y = 0x045;
constexpr uint16_t z = 0x04A;
} // namespace accel
namespace gyro {
constexpr uint16_t x = 0x050;
constexpr uint16_t y = 0x055;
constexpr uint16_t z = 0x05A;
} // namespace accel
} // namespace mpu6050
namespace mpu9255 {
namespace accel {
constexpr uint16_t x = 0x060;
constexpr uint16_t y = 0x065;
constexpr uint16_t z = 0x06A;
} // namespace accel
namespace gyro {
constexpr uint16_t x = 0x070;
constexpr uint16_t y = 0x075;
constexpr uint16_t z = 0x07A;
} // namespace accel
} // namespace mpu9255

namespace stats {
namespace int_motors {
constexpr uint16_t _ndx = 0x080;
constexpr uint16_t rec1 = 0x090;
constexpr uint16_t rec2 = 0x094;
constexpr uint16_t rec3 = 0x098;
constexpr uint16_t rec4 = 0x09C;
constexpr uint16_t rec5 = 0x0A0;
constexpr uint16_t rec6 = 0x0A4;
constexpr uint16_t rec7 = 0x0A8;
constexpr uint16_t rec8 = 0x0AC;
} // namespace int_motors
} // namespace stats


const uint16_t rateD = 995;
const uint16_t rateP = 1000;
const uint16_t rateYawP = 1005;
const uint16_t rateYawI = 1010;
const uint16_t angleP = 1015;
const uint16_t angleI = 1020;
} // namespace eeprom


namespace ctlr {
namespace att {
namespace rate {
extern float P;
extern float I;
extern float D;
const int16_t output_limit = 70;
namespace yaw {
extern float P;
extern float I;
const uint32_t update_rate = 2600;
const uint8_t integral_limit = 10;
} // namespace yaw
} // namespace rate
namespace angle {
extern float P;
extern float I;
const uint32_t update_rate = 10560;
const int16_t output_limit = 250;
const uint8_t integral_limit = 20;
} // namespace angle
} // namespace att
} // namespace ctlr


namespace mix {
const uint8_t minimum_regulation_throttle = 5;
const uint8_t maximum_base_throttle = 115;
} // namespace mix


namespace indication {
const uint32_t period = 200000;
extern uint8_t arms_level;
const bool lamp = true;
} // namespace indication

namespace battery {
const uint32_t update_rate = 200000;
const float alpha = 0.7f;
const float low_voltage = 10.5f;
const float critical_voltage = 10.0f;
} // namespace battery

void init();
} // namespace cfg
