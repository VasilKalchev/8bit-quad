/*
The MIT License (MIT)

Copyright (c) 2016 Vasil Kalchev

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "MPU925x_I2C.hpp"


MPU925x_I2C::MPU925x_I2C(uint8_t address) : _i2cAddress(address) {}

void MPU925x_I2C::initialize() {
  setSleepMode(false);
  setClockSource(1);
  setRate(1);
  setAccelDLPF(0);
  setGyroDLPF(0);
  setGyroFullScaleRange(1);
  setAccelFullScaleRange(1);
  I2C::writeBit(_i2cAddress, MPU925x_RA_INT_PIN_CFG, true, MPU925x_BYPASS_EN_BIT);
  I2C::writeBit(_i2cAddress, MPU925x_RA_USER_CTRL, false, MPU925x_I2C_MST_EN_BIT);
}

uint8_t MPU925x_I2C::getDeviceId() const {
  uint8_t buffer;
  I2C::readBits(_i2cAddress, MPU925x_RA_WHO_AM_I, &buffer, MPU925x_WHOAMI_BIT, MPU925x_WHOAMI_LENGTH);
  return buffer;
}

bool MPU925x_I2C::testConnection() const {
  const uint8_t id = getDeviceId();
  return (id == MPU9250_ID || id == MPU9255_ID);
}



int16_t MPU925x_I2C::getXGyroOffset() {
  I2C::readBytes(_i2cAddress, MPU925x_RA_XG_OFFSET_H, _buffer, 2);
  return (((int16_t)_buffer[0]) << 8) | _buffer[1];
}
bool MPU925x_I2C::setXGyroOffset(int16_t offset) {
  return I2C::writeWord(_i2cAddress, MPU925x_RA_XG_OFFSET_H, offset);
}

int16_t MPU925x_I2C::getYGyroOffset() {
  I2C::readBytes(_i2cAddress, MPU925x_RA_YG_OFFSET_H, _buffer, 2);
  return (((int16_t)_buffer[0]) << 8) | _buffer[1];
}
bool MPU925x_I2C::setYGyroOffset(int16_t offset) {
  return I2C::writeWord(_i2cAddress, MPU925x_RA_YG_OFFSET_H, offset);
}

int16_t MPU925x_I2C::getZGyroOffset() {
  I2C::readBytes(_i2cAddress, MPU925x_RA_ZG_OFFSET_H, _buffer, 2);
  return (((int16_t)_buffer[0]) << 8) | _buffer[1];
}
bool MPU925x_I2C::setZGyroOffset(int16_t offset) {
  return I2C::writeWord(_i2cAddress, MPU925x_RA_ZG_OFFSET_H, offset);
}

int16_t MPU925x_I2C::getXAccelOffset() {
  I2C::readBytes(_i2cAddress, MPU925x_RA_XA_OFFSET_H, _buffer, 2);
  return ((((int16_t)_buffer[0]) << 8) | _buffer[1]) >> 1;
}
bool MPU925x_I2C::setXAccelOffset(int16_t offset) {
  return I2C::writeWord(_i2cAddress, MPU925x_RA_XA_OFFSET_H, offset << 1);
}

int16_t MPU925x_I2C::getYAccelOffset() {
  I2C::readBytes(_i2cAddress, MPU925x_RA_YA_OFFSET_H, _buffer, 2);
  return ((((int16_t)_buffer[0]) << 8) | _buffer[1]) >> 1;
}
bool MPU925x_I2C::setYAccelOffset(int16_t offset) {
  return I2C::writeWord(_i2cAddress, MPU925x_RA_YA_OFFSET_H, offset << 1);
}

int16_t MPU925x_I2C::getZAccelOffset() {
  I2C::readBytes(_i2cAddress, MPU925x_RA_ZA_OFFSET_H, _buffer, 2);
  return ((((int16_t)_buffer[0]) << 8) | _buffer[1]) >> 1;
}
bool MPU925x_I2C::setZAccelOffset(int16_t offset) {
  return I2C::writeWord(_i2cAddress, MPU925x_RA_ZA_OFFSET_H, offset << 1);
}

uint8_t MPU925x_I2C::getGyroFullScaleRange() {
  I2C::readBits(_i2cAddress, MPU925x_RA_GYRO_CONFIG, _buffer, MPU925x_GYRO_FS_SEL_BIT, MPU925x_GYRO_FS_SEL_LENGTH);
  return _buffer[0];
}
bool MPU925x_I2C::setGyroFullScaleRange(uint8_t scale) {
  if (scale == 0b00) {
    _gyroscopeSensitivity = 131.0; // LSB/(deg/s), Range = +-250deg/s
  } else if (scale == 0b01) {
    _gyroscopeSensitivity = 65.5; // LSB/(deg/s), Range = +-500deg/s
  } else if (scale == 0b10) {
    _gyroscopeSensitivity = 32.8; // LSB/(deg/s), Range = +-1000deg/s
  } else if (scale == 0b11) {
    _gyroscopeSensitivity = 16.4; // LSB/(deg/s), Range = +-2000deg/s
  } else {
    return false;
  }
  return I2C::writeBits(_i2cAddress, MPU925x_RA_GYRO_CONFIG, scale, MPU925x_GYRO_FS_SEL_BIT, MPU925x_GYRO_FS_SEL_LENGTH);
}
uint8_t MPU925x_I2C::getAccelFullScaleRange() {
  I2C::readBits(_i2cAddress, MPU925x_RA_ACCEL_CONFIG, _buffer, MPU925x_ACCEL_FS_SEL_BIT, MPU925x_ACCEL_FS_SEL_LENGTH);
  return _buffer[0];
}
bool MPU925x_I2C::setAccelFullScaleRange(uint8_t scale) {
  if (scale == 0b00) {
    _accelerometerSensitivity = 16384; // LSB/g, Range = 1g
  } else if (scale == 0b01) {
    _accelerometerSensitivity = 8192; // LSB/g, Range = 2g
  } else if (scale == 0b10) {
    _accelerometerSensitivity = 4096; // LSB/g, Range = 4g
  } else if (scale == 0b11) {
    _accelerometerSensitivity = 2048; // LSB/g, Range = 8g
  } else {
    return false;
  }
  return I2C::writeBits(_i2cAddress, MPU925x_RA_ACCEL_CONFIG, scale, MPU925x_ACCEL_FS_SEL_BIT, MPU925x_ACCEL_FS_SEL_LENGTH);
}

uint8_t MPU925x_I2C::getRate() {
  I2C::readByte(_i2cAddress, MPU925x_RA_SMPLRT_DIV, _buffer);
  return _buffer[0];
}
bool MPU925x_I2C::setRate(uint8_t rate) {
  return I2C::writeByte(_i2cAddress, MPU925x_RA_SMPLRT_DIV, rate);
}

int8_t MPU925x_I2C::getGyroDLPF() {
  I2C::readBits(_i2cAddress, MPU925x_RA_GYRO_CONFIG, _buffer, MPU925x_FCHOICE_B_BIT, MPU925x_FCHOICE_B_LENGTH);
  uint8_t fchoice = _buffer[0];
  I2C::readBits(_i2cAddress, MPU925x_RA_CONFIG, _buffer, MPU925x_DLPF_CFG_SET_BIT, MPU925x_DLPF_CFG_SET_LENGTH);
  if (fchoice == 0b00) {
    return _buffer[0];
  } else if (fchoice == 0b10) {
    return -1;
  } else {
    return -2;
  }
}
bool MPU925x_I2C::setGyroDLPF(int8_t dlpf) {
  if (dlpf > 7) return false;
  uint8_t fchoice = 0b00;
  if (dlpf == -2) {
    fchoice = 0b01;
  } else if (dlpf == -1) {
    fchoice = 0b10;
  } else {
    fchoice = 0b00;
  }
  I2C::writeBits(_i2cAddress, MPU925x_RA_GYRO_CONFIG, fchoice, MPU925x_FCHOICE_B_BIT, MPU925x_FCHOICE_B_LENGTH);
  return I2C::writeBits(_i2cAddress, MPU925x_RA_CONFIG, dlpf, MPU925x_DLPF_CFG_SET_BIT, MPU925x_DLPF_CFG_SET_LENGTH);
}
int8_t MPU925x_I2C::getAccelDLPF() {
  bool fchoice;
  I2C::readBit(_i2cAddress, MPU925x_RA_ACCEL_CONFIG_2, &fchoice, MPU925x_ACCEL_FCHOICE_B_BIT);
  I2C::readBits(_i2cAddress, MPU925x_RA_ACCEL_CONFIG_2, _buffer, MPU925x_A_DLPF_CFG_BIT, MPU925x_A_DLPF_CFG_LENGTH);
  if (fchoice == 0b0) return _buffer[0];
  else return -1;
  // I2C::readBits(_i2cAddress, MPU925x_RA_ACCEL_CONFIG_2, _buffer, 1, 2);
  // return _buffer[0];
}
bool MPU925x_I2C::setAccelDLPF(int8_t dlpf) {
  if (dlpf > 7) return false;
  uint8_t fchoice = 0b0;
  if (dlpf == -1) {
    fchoice = 0b1;
  } else {
    fchoice = 0b0;
  }
  I2C::writeBit(_i2cAddress, MPU925x_RA_ACCEL_CONFIG_2, fchoice, MPU925x_ACCEL_FCHOICE_B_BIT);
  return I2C::writeBits(_i2cAddress, MPU925x_RA_ACCEL_CONFIG_2, dlpf, MPU925x_A_DLPF_CFG_BIT, MPU925x_A_DLPF_CFG_LENGTH);
  // return I2C::writeByte(_i2cAddress, MPU925x_RA_ACCEL_CONFIG_2, 0b00000011, 1, 2);
}

uint8_t MPU925x_I2C::getClockSource() {
  I2C::readBits(_i2cAddress, MPU925x_RA_PWR_MGMT_1, _buffer, MPU925x_CLKSEL_BIT, MPU925x_CLKSEL_LENGTH);
  return _buffer[0];
}
bool MPU925x_I2C::setClockSource(uint8_t clockSource) {
  return I2C::writeBits(_i2cAddress, MPU925x_RA_PWR_MGMT_1, clockSource, MPU925x_CLKSEL_BIT, MPU925x_CLKSEL_LENGTH);
}

bool MPU925x_I2C::getSleepMode() {
  bool sleepMode;
  I2C::readBit(_i2cAddress, MPU925x_RA_PWR_MGMT_1, &sleepMode, MPU925x_SLEEP_BIT);
  return sleepMode;
}
bool MPU925x_I2C::setSleepMode(bool enabled) {
  return I2C::writeBit(_i2cAddress, MPU925x_RA_PWR_MGMT_1, enabled, MPU925x_SLEEP_BIT);
}

bool MPU925x_I2C::getAccelRaw(int16_t* x, int16_t* y, int16_t* z) {
  if (I2C::readBytes(_i2cAddress, MPU925x_RA_ACCEL_XOUT_H, _buffer, 6)) {
    *x = ((int16_t)_buffer[0] << 8) | _buffer[1];
    *y = ((int16_t)_buffer[2] << 8) | _buffer[3];
    *z = ((int16_t)_buffer[4] << 8) | _buffer[5];
    return true;
  } else {
    return false;
  }
}

bool MPU925x_I2C::getAccel(float* x, float* y, float* z) {
  int16_t xi;
  int16_t yi;
  int16_t zi;
  if (getAccelRaw(&xi, &yi, &zi) ) {
    *x = xi / _accelerometerSensitivity;
    *y = yi / _accelerometerSensitivity;
    *z = zi / _accelerometerSensitivity;
    return true;
  } else {
    return false;
  }
}

bool MPU925x_I2C::getAccelXRaw(int16_t* x) {
  if (I2C::readBytes(_i2cAddress, MPU925x_RA_ACCEL_XOUT_H, _buffer, 2)) {
    *x = ((uint16_t)_buffer[0] << 8) | _buffer[1];
    return true;
  } else {
    return false;
  }
}

bool MPU925x_I2C::getAccelYRaw(int16_t* y) {
  if (I2C::readBytes(_i2cAddress, MPU925x_RA_ACCEL_YOUT_H, _buffer, 2)) {
    *y = ((uint16_t)_buffer[0] << 8) | _buffer[1];
    return true;
  } else {
    return false;
  }
}
bool MPU925x_I2C::getAccelZRaw(int16_t* z) {
  if (I2C::readBytes(_i2cAddress, MPU925x_RA_ACCEL_ZOUT_H, _buffer, 2)) {
    *z = ((uint16_t)_buffer[0] << 8) | _buffer[1];
    return true;
  } else {
    return false;
  }
}

bool MPU925x_I2C::getGyroRaw(int16_t* x, int16_t* y, int16_t* z) {
  if (I2C::readBytes(_i2cAddress, MPU925x_RA_GYRO_XOUT_H, _buffer, 6)) {
    *x = ((int16_t)_buffer[0] << 8) | _buffer[1];
    *y = ((int16_t)_buffer[2] << 8) | _buffer[3];
    *z = ((int16_t)_buffer[4] << 8) | _buffer[5];
    return true;
  } else {
    return false;
  }
}

bool MPU925x_I2C::getGyro(float* x, float* y, float* z) {
  int16_t xi;
  int16_t yi;
  int16_t zi;
  if (getGyroRaw(&xi, &yi, &zi) ) {
    *x = xi / _gyroscopeSensitivity;
    *y = yi / _gyroscopeSensitivity;
    *z = zi / _gyroscopeSensitivity;
    return true;
  } else {
    return false;
  }
}

bool MPU925x_I2C::getTemperatureRaw(uint16_t* temperature) {
  if (I2C::readBytes(_i2cAddress, MPU925x_RA_TEMP_OUT_H, _buffer, 2)) {
    *temperature = ((uint16_t)_buffer[0] << 8) | _buffer[1];
    return true;
  } else {
    return false;
  }
}

bool MPU925x_I2C::getTemperature(float* temperature) {
  int16_t temperature_i;
  if (getTemperatureRaw(&temperature_i)) {
    *temperature = temperature_i / 333.97 + 21.0;
    // *temperature = temperature_i / 340.0 + 36.53;
    return true;
  }
  return false;
}

bool MPU925x_I2C::getAccelAndGyroRaw(int16_t* ax, int16_t* ay, int16_t* az,
                                     int16_t* gx, int16_t* gy, int16_t* gz) {
  if (I2C::readBytes(_i2cAddress, MPU925x_RA_ACCEL_XOUT_H, _buffer, 14)) {
    *ax = ((uint16_t)_buffer[0] << 8) | _buffer[1];
    *ay = ((uint16_t)_buffer[2] << 8) | _buffer[3];
    *az = ((uint16_t)_buffer[4] << 8) | _buffer[5];
    *gx = ((uint16_t)_buffer[8] << 8) | _buffer[9];
    *gy = ((uint16_t)_buffer[10] << 8) | _buffer[11];
    *gz = ((uint16_t)_buffer[12] << 8) | _buffer[13];
    return true;
  } else {
    return false;
  }
}


void MPU925x_I2C::getFIFOBytes(uint8_t *data, uint8_t length) {
  I2C::readBytes(_i2cAddress, MPU925x_RA_FIFO_R_W, data, length);
}


float MPU925x_I2C::accelRawToG(const int16_t accelRaw) const {
  return (float)accelRaw / _accelerometerSensitivity;
}

float MPU925x_I2C::accelRawToMs2(const int16_t accelRaw) const {
  return ((float)accelRaw / _accelerometerSensitivity) * 9.80665;
}

float MPU925x_I2C::gyroRawToDegS(const int16_t gyroRaw) const {
  return (float)gyroRaw / (float)_gyroscopeSensitivity;
}

float MPU925x_I2C::gyroRawToRadS(const int16_t gyroRaw) const {
  return ((float)gyroRaw / (float)_gyroscopeSensitivity) * 0.01745329251;
}
