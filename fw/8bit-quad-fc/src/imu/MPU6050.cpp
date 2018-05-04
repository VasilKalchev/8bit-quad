#include "MPU6050.hpp"



MPU6050_i2cDev::MPU6050_i2cDev(uint8_t address)
  : _mpu6050(address), _sensitivity(1) {}

bool MPU6050_i2cDev::initialize() {
  // Wire.begin();
  // Wire.setClock(800000); // max: 888888
  // Fastwire::setup(800, true);
  // delay(10);


  _mpu6050.initialize();
  for (auto i = 0; i < 2; ++i) {

// DLPF
//          |   ACCELEROMETER    |           GYROSCOPE
// DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
// ---------+-----------+--------+-----------+--------+-------------
// 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
// 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
// 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
// 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
// 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
// 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
// 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
// 7        |   -- Reserved --   |   -- Reserved --   | Reserved

// Full scale gyro range
// FS_SEL | Full Scale Range   | LSB Sensitivity
// -------+--------------------+----------------
// 0      | +/- 250 degrees/s  | 131 LSB/deg/s
// 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
// 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
// 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s

// Full scale accel range
// AFS_SEL | Full Scale Range | LSB Sensitivity
// --------+------------------+----------------
// 0       | +/- 2g           | 16384 LSB/mg
// 1       | +/- 4g           | 8192 LSB/mg
// 2       | +/- 8g           | 4096 LSB/mg
// 3       | +/- 16g          | 2048 LSB/mg

    setDLPFMode(1);
    setSensitivity(1);

    _mpu6050.setXAccelOffset(-5400); //19
    _mpu6050.setYAccelOffset(-6073); //130
    _mpu6050.setZAccelOffset(1526);
    _mpu6050.setXGyroOffset(77);
    _mpu6050.setYGyroOffset(-24);
    _mpu6050.setZGyroOffset(11);
  }
  return _mpu6050.testConnection();
}

void MPU6050_i2cDev::setDLPFMode(int8_t mode) {
  if (mode < 0) mode = 0;
  else if (mode > 6) mode = 6;
  _mpu6050.setDLPFMode(mode);
}

void MPU6050_i2cDev::setSensitivity(int8_t sensitivity) {
  if (sensitivity < 0) sensitivity = 0;
  else if (sensitivity > 3) sensitivity = 3;
  _mpu6050.setFullScaleGyroRange(sensitivity);
  _mpu6050.setFullScaleAccelRange(sensitivity);
  _sensitivity = sensitivity;
}

float MPU6050_i2cDev::toGForce(int16_t acceleration) {
  return acceleration / MPU6050_i2cDev::_accelSensitivity[_sensitivity];
}

void MPU6050_i2cDev::getAngularVelocity(AngularVelocity* angularVelocity) {
  int16_t xg, yg, zg;
  _mpu6050.getRotation(&xg, &yg, &zg);
  angularVelocity->y = -xg / 65.5;
  angularVelocity->x = 0 - (yg / 65.5);
  angularVelocity->z = zg / 65.5;
}

void MPU6050_i2cDev::getAcceleration(Acceleration* acceleration) {
  _mpu6050.getAcceleration(&acceleration->y, &acceleration->x, &acceleration->z);
}

void MPU6050_i2cDev::getMotion(AngularVelocity* angularVelocity, Acceleration* acceleration) {
  int16_t xg, yg, zg;
  Acceleration accelTemp;
  _mpu6050.getMotion6(&accelTemp.x, &accelTemp.y, &accelTemp.z, &xg, &yg, &zg);
  if (accelTemp.y == 0 && accelTemp.z == 0 && xg == 0 && yg == 0) return;
  acceleration->x = accelTemp.x;
  acceleration->y = accelTemp.y;
  acceleration->z = accelTemp.z;
  angularVelocity->x = xg / 65.5;
  angularVelocity->y = yg / 65.5;
  angularVelocity->z = zg / 65.5;
}

void MPU6050_i2cDev::getRawMotion(int16_t* xa, int16_t* ya, int16_t* za, int16_t* xg, int16_t* yg, int16_t* zg) {
  _mpu6050.getMotion6(xa, ya, za, xg, yg, zg);
}

float MPU6050_i2cDev::getTemperature() {
  return (float)_mpu6050.getTemperature() / 340.0 + 36.53;
}

void MPU6050_i2cDev::updateOffsets() {
  float temperature = getTemperature();
  if (temperature > 0 && temperature < 15) {

  }
  _mpu6050.setXAccelOffset(-5258);
  _mpu6050.setYAccelOffset(-6012);
  _mpu6050.setZAccelOffset(1478);
  _mpu6050.setXGyroOffset(73);
  _mpu6050.setYGyroOffset(-15);
  _mpu6050.setZGyroOffset(-9);
}

float MPU6050_i2cDev::_gyroSensitivity[] = {131.0, 65.5, 32.8, 16.4};
int16_t MPU6050_i2cDev::_accelSensitivity[] = {16384, 8192, 4096, 2048};