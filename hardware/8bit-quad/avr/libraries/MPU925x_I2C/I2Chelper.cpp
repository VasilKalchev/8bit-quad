#include "I2Chelper.hpp"

bool I2C::writeBit(uint8_t slaveAddress, uint8_t registerAddress, bool bit, uint8_t position) {
  uint8_t byte;
  if (readByte(slaveAddress, registerAddress, &byte)) {
    byte = (bit != 0) ? (byte | (1 << position)) : (byte & ~(1 << position));
    return writeByte(slaveAddress, registerAddress, byte);
  } else {
    return false;
  }
}

bool I2C::writeBits(uint8_t slaveAddress, uint8_t registerAddress, uint8_t data, uint8_t bitStart, uint8_t length) {
  uint8_t byte;
  if (readByte(slaveAddress, registerAddress, &byte)) {
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1);
    data &= mask;
    byte &= ~(mask);
    byte |= data;
    return writeByte(slaveAddress, registerAddress, byte);
  } else {
    return false;
  }
}

bool I2C::readBit(uint8_t slaveAddress, uint8_t registerAddress, bool* bit, uint8_t position) {
  uint8_t byte;
  bool success = readByte(slaveAddress, registerAddress, &byte);
  *bit = byte & (1 << position);
  return success;
}

bool I2C::readBits(uint8_t slaveAddress, uint8_t registerAddress, uint8_t* data, uint8_t bitStart, uint8_t length) {
  uint8_t byte;
  bool success = readByte(slaveAddress, registerAddress, &byte);
  uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
  byte &= mask;
  byte >>= (bitStart - length + 1);
  *data = byte;
  return success;
}

bool I2C::writeByte(uint8_t slaveAddress, uint8_t registerAddress, uint8_t data) {
  return writeBytes(slaveAddress, registerAddress, &data, 1);
}

bool I2C::writeBytes(uint8_t slaveAddress, uint8_t registerAddress, uint8_t* data, uint8_t length) {
  Wire.beginTransmission(slaveAddress);
  Wire.write(registerAddress);
  for (uint8_t i = 0; i < length; ++i) {
    Wire.write((uint8_t)data[i]);
  }
  return Wire.endTransmission() == 0;
}

bool I2C::readByte(uint8_t slaveAddress, uint8_t registerAddress, uint8_t* data) {
  return readBytes(slaveAddress, registerAddress, data, 1);
}

uint8_t I2C::readBytes(uint8_t slaveAddress, uint8_t registerAddress, uint8_t* data, uint8_t length) {
  uint32_t startTime = micros();
  if (length > 31) return 0;
  Wire.beginTransmission(slaveAddress);
  Wire.write(registerAddress);
  Wire.endTransmission();
  Wire.requestFrom(slaveAddress, length);
  uint8_t i = 0;
  while (Wire.available() && (micros() < startTime + _timeout)) {
    data[i++] = Wire.read();
  }
  if (micros() > startTime + _timeout && i < length) i = 0;
  return i;
}

bool I2C::writeWord(uint8_t slaveAddress, uint8_t registerAddress, uint16_t data) {
  return writeWords(slaveAddress, registerAddress, &data, 1);
}

bool I2C::writeWords(uint8_t slaveAddress, uint8_t registerAddress, uint16_t* data, uint8_t length) {
  Wire.beginTransmission(slaveAddress);
  Wire.write(registerAddress);
  for (uint8_t i = 0; i < length * 2; i++) {
    Wire.write((uint8_t)(data[i] >> 8));
    Wire.write((uint8_t)data[i++]);
  }
  return Wire.endTransmission() == 0;
}

bool I2C::setTimeout(uint32_t timeout) {
  _timeout = timeout;
  return true;
}

uint32_t I2C::_timeout = 1000; // usec