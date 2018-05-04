#pragma once
#include <inttypes.h>

#include "Wire.h"
#include <MPU925x_I2C.hpp>
#include "../common/imu_types.hpp"


class MPU9255 {
public:
  MPU9255(uint8_t address = 0x73);

  bool initialize();
  void getMotion(AngularVelocity* angularVelocity, Acceleration* acceleration);
  float getTemperature();

  MPU925x_I2C _mpu9255;
private:
};
