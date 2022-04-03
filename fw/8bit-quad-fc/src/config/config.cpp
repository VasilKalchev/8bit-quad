#include "config.hpp"

#include <avr/eeprom.h>


// #if FUSION_COMPLEMENTARY_FILTER == 0 && FUSION_MAHONY_FILTER == 0
//   #error "No 'fusion method' specified!"
// #endif

// #if ANGULAR_RATE_DIRECT_FROM_GYROSCOPE == 0 && ANGULAR_RATE_DIFFERENTIATE_FROM_MAHONY == 0
//   #error "No 'angular rate source' specified!"
// #endif


namespace cfg {

namespace imu {
bool median_filter_acc = false;

namespace offset {
#if defined(IMU_MPU6050)
int16_t accel[3] = {-5424, -5998, 1419};
int16_t gyro[3] = {62, -62, 9};
#elif defined(IMU_MPU925x)
int16_t accel[3] = {2576, -2493, 6261};
int16_t gyro[3] = {29, -9, 49};
#endif
} // namespace offset
} // namespace imu


namespace ctlr {
namespace att {
namespace angle {
  float P = 1.7; // 1.8
  float I = 0.35; // 0.6
} // namespace angle
namespace rate {
  float P = 0.25;
  float I = 0.0;
  float D = 0.07;
  // float yawP = 0.3; // 0.3
  // float yawI = 0.0; // 0.04
namespace yaw {
  float P = 0.3; // 0.3
  float I = 0.0; // 0.04
} // namespace yaw
} // namespace rate
} // namespace att
} // namespace ctlr

namespace indication {
uint8_t arms_level = 12;
} //namespace indication

namespace battery {
} //namespace battery

void init() {
/* EEPROM map
 * 
 *     | 0 1 2 3 4 5 6 7 8 9 A B C D E F |
 *     |---------------------------------|
 * 00x | p p p p i i i i d d d d p p p p | rate P, rate I, rate D, yaw rate P
 * 01x | i i i i d d d d p p p p i i i i | yaw rate I, yaw rate D, angle P, angle I
 * 02x | p p p p i i i i f f f f m m m m | mahony P, mahony I, accel LPF, IMU LPF
 * 03x | a l . . . . . . . . . . . . . . | arms LEDs, lamp
 * 04x | x x x x . y y y y . z z z z . . | MPU6050 accel offset
 * 05x | x x x x . y y y y . z z z z . . | MPU6050 gyro offset
 * 06x | x x x x . y y y y . z z z z . . | MPU9255 accel offset
 * 07x | x x x x . y y y y . z z z z . . | MPU9255 gyro offset
 * 08x | n . . . . . . . . . . . . . . . | Int(motor) ndx
 * 09x | 1 1 1 1 2 2 2 2 3 3 3 3 4 4 4 4 | Int(motor) 1, 2, 3, 4
 * 0Ax | 5 5 5 5 6 6 6 6 7 7 7 7 8 8 8 8 | Int(motor) 5, 6, 7, 8
 * 0Bx | . . . . . . . . . . . . . . . . |
 * 0Cx | . . . . . . . . . . . . . . . . |
 * 0Dx | . . . . . . . . . . . . . . . . |
 * 01x | . . . . . . . . . . . . . . . . |
 * 0Ex | . . . . . . . . . . . . . . . . |
 * 0Fx | . . . . . . . . . . . . . . . . |
 * 100 - 2FF ...
 * 30x | . . . . . . . . . . . . . . . . |
 * 31x | . . . . . . . . . . . . . . . . |
 * 32x | . . . . . . . . . . . . . . . . |
 * 33x | . . . . . . . . . . . . . . . . |
 * 34x | . . . . . . . . . . . . . . . . |
 * 35x | . . . . . . . . . . . . . . . . |
 * 36x | . . . . . . . . . . . . . . . . |
 * 37x | . . . . . . . . . . . . . . . . |
 * 38x | . . . . . . . . . . . . . . . . |
 * 39x | . . . . . . . . . . . . . . . . |
 * 3Ax | . . . . . . . . . . . . . . . . |
 * 3Bx | . . . . . . . . . . . . . . . . |
 * 3Cx | . . . . . . . . . . . . . . . . |
 * 3Dx | . . . . . . . . . . . . . . . . |
 * 3Ex | . . . x x x x . x x x x . x x x | rateD rateP yaw_rateP(3/4)
 * 3Fx | x . x x x x . x x x x . x x x x | yaw_rateP(1/4) yaw_rateI angleP angleI

Integrated motor output:
Int_motor += tl+tr+bl+br;

Max acceleration (x, y, z), min RMS acceleration
Max angular velocity (x, y, z)


 */


  #if RESET_EEPROM_TO_DEFAULT
  while (!eeprom_is_ready());
  eeprom_write_float((float*)eeprom::rateD,
    (float)ctlr::att::rate::D);

  while (!eeprom_is_ready());
  eeprom_write_float((float*)eeprom::rateP,
    (float)ctlr::att::rate::P);

  while (!eeprom_is_ready());
  eeprom_write_float((float*)eeprom::rateYawP,
    (float)ctlr::att::rate::yaw::P);

  while (!eeprom_is_ready());
  eeprom_write_float((float*)eeprom::rateYawI,
    (float)ctlr::att::rate::yaw::I);

  while (!eeprom_is_ready());
  eeprom_write_float((float*)eeprom::angleP,
    (float)ctlr::att::angle::P);

  while (!eeprom_is_ready());
  eeprom_write_float((float*)eeprom::angleI,
    (float)ctlr::att::angle::I);
  #endif


  float tmp = -123.456;
  while (!eeprom_is_ready());
  tmp = (float)eeprom_read_float((float*)eeprom::rateD);
  if (tmp >= 0.0 && tmp < 0.5) {
    ctlr::att::rate::D = tmp;
  }

  while (!eeprom_is_ready());
  tmp = (float)eeprom_read_float((float*)eeprom::rateP);
  if (tmp > 0.1 && tmp < 0.4) {
    ctlr::att::rate::P = tmp;
  }

  while (!eeprom_is_ready());
  tmp = (float)eeprom_read_float((float*)eeprom::rateYawP);
  if (tmp > 0.2 && tmp < 0.4) {
    ctlr::att::rate::yaw::P = tmp;
  }

  while (!eeprom_is_ready());
  tmp = (float)eeprom_read_float((float*)eeprom::rateYawI);
  if (tmp >= 0.0 && tmp < 0.2) {
    ctlr::att::rate::yaw::I = tmp;
  }

  while (!eeprom_is_ready());
  tmp = (float)eeprom_read_float((float*)eeprom::angleP);
  if (tmp > 1.4 && tmp < 2.4) {
    ctlr::att::angle::P = tmp;
  }

  while (!eeprom_is_ready());
  tmp = (float)eeprom_read_float((float*)eeprom::angleI);
  if (tmp >= 0.01 && tmp < 1.0) {
    ctlr::att::angle::I = tmp;
  }
}

} // namespace cfg
