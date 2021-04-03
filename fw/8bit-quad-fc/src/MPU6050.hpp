#pragma once
#include <inttypes.h>

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "IMU.hpp"


class MPU6050_i2cDev {
public:
  MPU6050_i2cDev(uint8_t address = 0x68);

  bool initialize();

  void setDLPFMode(int8_t mode);
  void setSensitivity(int8_t sensitivity);
  float toGForce(int16_t acceleration);

  void getAngularVelocity(AngularVelocity* angularVelocity);
  void getAcceleration(Acceleration* acceleration);
  void getMotion(AngularVelocity* angularVelocity, Acceleration* acceleration);
  void getRawMotion(int16_t* xa, int16_t* ya, int16_t* za, int16_t* xg, int16_t* yg, int16_t* zg);

  float getTemperature();
  void updateOffsets();

  MPU6050 _mpu6050;
private:
  int8_t _sensitivity;
  static float _gyroSensitivity[];
  static int16_t _accelSensitivity[];
};
