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

    // _mpu9255.setXAccelOffset(2587);
    // _mpu9255.setYAccelOffset(-2474);
    // _mpu9255.setZAccelOffset(5226);
    // _mpu9255.setXGyroOffset (6);
    // _mpu9255.setYGyroOffset (-7);
    // _mpu9255.setZGyroOffset (2);


    _mpu9255.setXAccelOffset(2594);
    _mpu9255.setYAccelOffset(-2479);
    _mpu9255.setZAccelOffset(5227);
    _mpu9255.setXGyroOffset (34);
    _mpu9255.setYGyroOffset (-6);
    _mpu9255.setZGyroOffset (66);

/*
|  Temp |   ax  |   ay  |   az  |   gx  |   gy  |   gz  |
|-------|-------|-------|-------|-------|-------|-------|
| 12.00 |  xxxx | -xxxx |  xxxx |    25 |    -8 |    72 |
| 13.33 |  2607 | -2491 |  6247 |    26 |    -9 |    70 |
| 13.86 |  2608 | -2491 |  6244 |    26 |    -8 |    69 |
| 14.87 |  2608 | -2491 |  6243 |    27 |    -8 |    68 |
| 15.63 |  2609 | -2491 |  6242 |    27 |    -8 |    67 |
| 15.78 |  2609 | -2491 |  6242 |    27 |    -8 |    68 |
| 15.92 |  2609 | -2491 |  6242 |    27 |    -8 |    67 |
| 16.15 |  2609 | -2491 |  6242 |    28 |    -8 |    67 |
| 16.35 |  2609 | -2491 |  6242 |    28 |    -8 |    67 |
| 16.45 |  2609 | -2491 |  6242 |    28 |    -8 |    67 |
| 16.30 |  2609 | -2491 |  6242 |    28 |    -8 |    67 |
| 16.35 |  2609 | -2491 |  6245 |    28 |    -7 |    67 |
| 16.45 |  2609 | -2491 |  6246 |    28 |    -7 |    67 |
| 17.07 |  2610 | -2491 |  6245 |    29 |    -7 |    67 |
| 18.41 |  2610 | -2492 |  6242 |    30 |    -5 |    66 |
| 19.00 |  2611 | -2492 |  6241 |    30 |    -6 |    66 |
| 19.13 |  2611 | -2492 |  6244 |    27 |    -7 |    63 |
| xxxxx |  xxxx | -xxxx |  xxxx |    xx |    -x |    xx |
| xxxxx |  xxxx | -xxxx |  xxxx |    xx |    -x |    xx |
| xxxxx |  xxxx | -xxxx |  xxxx |    xx |    -x |    xx |
*/
  }
  s &= _mpu9255.testConnection();
  return s;
}

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