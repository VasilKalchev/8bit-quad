#include "MPU9255.hpp"

MPU9255::MPU9255(uint8_t address) : _mpu9255(address) {}

bool MPU9255::initialize() {
  delay(5);
  bool s = true;
  _mpu9255.initialize();
  s &= _mpu9255.setAccelDLPF(3);
  s &= _mpu9255.setAccelFullScaleRange(1);
  s &= _mpu9255.setGyroDLPF(2);
  s &= _mpu9255.setGyroFullScaleRange(1);

  // Cleanflight v1.0: 42Hz 4.8ms

  _mpu9255.setXAccelOffset(2576);
  _mpu9255.setYAccelOffset(-2493);
  _mpu9255.setZAccelOffset(6261);
  _mpu9255.setXGyroOffset (29);
  _mpu9255.setYGyroOffset (-9);
  _mpu9255.setZGyroOffset (49);

  s &= _mpu9255.testConnection();
  return s;
}

bool MPU9255::setAccelFilter(int8_t filter) {
    return _mpu9255.setAccelDLPF(filter);
}

bool MPU9255::setGyroFilter(int8_t filter) {
    return _mpu9255.setGyroDLPF(filter);
}


int16_t MPU9255::getXGyroOffset() {
  return _mpu9255.getXGyroOffset();
}

bool MPU9255::setXGyroOffset(int16_t offset) {
  return _mpu9255.setXGyroOffset(offset);
}

int16_t MPU9255::getYGyroOffset() {
  return _mpu9255.getYGyroOffset();
}

bool MPU9255::setYGyroOffset(int16_t offset) {
  return _mpu9255.setYGyroOffset(offset);
}

int16_t MPU9255::getZGyroOffset() {
  return _mpu9255.getZGyroOffset();
}

bool MPU9255::setZGyroOffset(int16_t offset) {
  return _mpu9255.setZGyroOffset(offset);
}

int16_t MPU9255::getXAccelOffset() {
  return _mpu9255.getXAccelOffset();
}

bool MPU9255::setXAccelOffset(int16_t offset) {
  return _mpu9255.setXAccelOffset(offset);
}

int16_t MPU9255::getYAccelOffset() {
  return _mpu9255.getYAccelOffset();
}

bool MPU9255::setYAccelOffset(int16_t offset) {
  return _mpu9255.setYAccelOffset(offset);
}

int16_t MPU9255::getZAccelOffset() {
  return _mpu9255.getZAccelOffset();
}

bool MPU9255::setZAccelOffset(int16_t offset) {
  return _mpu9255.setZAccelOffset(offset);
}



uint8_t MPU9255::getMotion(AngularVelocity* angularVelocity, Acceleration* acceleration) {
  int16_t xg = 0, yg = 0, zg = 0;
  Acceleration accelTemp;
  accelTemp.x = 0;
  accelTemp.y = 0;
  accelTemp.z = 0;

  bool s = _mpu9255.getAccelAndGyroRaw(
    &accelTemp.x, &accelTemp.y, &accelTemp.z,
    &xg, &yg, &zg
  );

  if (s == false) {
    return 2;
  }

  if (accelTemp.y == 0 && accelTemp.z == 0 && xg == 0 && yg == 0) return 1;

  acceleration->x = accelTemp.x;
  acceleration->y = accelTemp.y;
  acceleration->z = accelTemp.z;
  angularVelocity->x = xg / 65.5;
  angularVelocity->y = yg / 65.5;
  angularVelocity->z = zg / 65.5;

  return 0;
}

float MPU9255::getTemperature() {
  float temperature = 0.0f;
  _mpu9255.getTemperature(&temperature);
  return temperature;
}