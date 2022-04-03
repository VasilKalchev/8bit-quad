#pragma once
#include <stdint.h>

#include "Wire.h"
#include <MPU6050.h>
#include "imu_types.hpp"


class MPU6050_i2cDev {
public:
  MPU6050_i2cDev(uint8_t address = 0x68);

  bool initialize();

  void setDLPFMode(int8_t mode);
  void setSensitivity(int8_t sensitivity);
  float toGForce(int16_t acceleration);

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

  void getAngularVelocity(AngularVelocity* angularVelocity);
  void getAcceleration(Acceleration* acceleration);
  uint8_t getMotion(AngularVelocity* angularVelocity, Acceleration* acceleration);
  void getMotionFifo(AngularVelocity angularVelocity[], Acceleration acceleration[]);
  void getRawMotion(int16_t* xa, int16_t* ya, int16_t* za, int16_t* xg, int16_t* yg, int16_t* zg);

  float getTemperature();
  void updateOffsets();

  MPU6050 _mpu6050;
private:
  int8_t _sensitivity;
  static float _gyroSensitivity[];
  static int16_t _accelSensitivity[];
};
