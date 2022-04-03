#pragma once
#include <stdint.h>

#include <MPU925x_I2C.hpp>
#include "imu_types.hpp"


class MPU9255 {
public:
  MPU9255(uint8_t address = 0x73);

  bool initialize();
  bool setAccelFilter(int8_t filter);
  bool setGyroFilter(int8_t filter);
  
  int16_t getXGyroOffset();
  bool setXGyroOffset(int16_t offset);
  int16_t getYGyroOffset();
  bool setYGyroOffset(int16_t offset);
  int16_t getZGyroOffset();
  bool setZGyroOffset(int16_t offset);
  int16_t getXAccelOffset();
  bool setXAccelOffset(int16_t offset);
  int16_t getYAccelOffset();
  bool setYAccelOffset(int16_t offset);
  int16_t getZAccelOffset();
  bool setZAccelOffset(int16_t offset);

  uint8_t getMotion(AngularVelocity* angularVelocity, Acceleration* acceleration);
  float getTemperature();

  MPU925x_I2C _mpu9255;
private:
};
