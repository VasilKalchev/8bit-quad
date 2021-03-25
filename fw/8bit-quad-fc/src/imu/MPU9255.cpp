#include "MPU9255.hpp"

MPU9255::MPU9255(uint8_t address) : _mpu9255(address) {}

bool MPU9255::initialize() {
  delay(10);
  bool s = true;
  _mpu9255.initialize();
  for (auto i = 0; i < 3; ++i) {
    s &= _mpu9255.setAccelDLPF(4);
    s &= _mpu9255.setAccelFullScaleRange(1);
    s &= _mpu9255.setGyroDLPF(0);
    s &= _mpu9255.setGyroFullScaleRange(1);
    _mpu9255.setXAccelOffset(2606);
    _mpu9255.setYAccelOffset(-2476);
    _mpu9255.setZAccelOffset(6251);
    _mpu9255.setXGyroOffset (25);
    _mpu9255.setYGyroOffset (-3);
    _mpu9255.setZGyroOffset (64);
  }
  s &= _mpu9255.testConnection();
  return s;
}

// Returns: 0 - ok, 1 - zero read (sensor likely not ready yet),
// 2 - I2C failure (read did not complete).
void MPU9255::getMotion(AngularVelocity* angularVelocity, Acceleration* acceleration) {
  int16_t xg = 0, yg = 0, zg = 0;
  Acceleration accelTemp;
  accelTemp.x = 0;
  accelTemp.y = 0;
  accelTemp.z = 0;
  _mpu9255.getAccelAndGyroRaw(&accelTemp.x, &accelTemp.y, &accelTemp.z, &xg, &yg, &zg);
  if (accelTemp.y == 0 && accelTemp.z == 0 && xg == 0 && yg == 0) return;
  acceleration->x = accelTemp.x;
  acceleration->y = accelTemp.y;
  acceleration->z = accelTemp.z;
  angularVelocity->x = xg / 65.5;
  angularVelocity->y = yg / 65.5;
  angularVelocity->z = zg / 65.5;
}

float MPU9255::getTemperature() {
  float temperature = 0.0f;
  _mpu9255.getTemperature(&temperature);
  return temperature;
}