#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

// i2c and spi helper
// maybe base class for device class
// choose from arduino wire or other libraries

class I2C {
public:
  static bool readBit(uint8_t slaveAddress, uint8_t registerAddress, bool* bit, uint8_t position);
  static bool readBits(uint8_t slaveAddress, uint8_t registerAddress, uint8_t* data, uint8_t bitStart, uint8_t length);
  static bool writeBit(uint8_t slaveAddress, uint8_t registerAddress, bool bit, uint8_t position);
  static bool writeBits(uint8_t slaveAddress, uint8_t registerAddress, uint8_t data, uint8_t bitStart, uint8_t length);

  static bool readByte(uint8_t slaveAddress, uint8_t registerAddress, uint8_t* data);
  static uint8_t readBytes(uint8_t slaveAddress, uint8_t registerAddress, uint8_t* data, uint8_t length);
  static bool writeByte(uint8_t slaveAddress, uint8_t registerAddress, uint8_t data);
  static bool writeBytes(uint8_t slaveAddress, uint8_t registerAddress, uint8_t* data, uint8_t length);

  static bool writeWord(uint8_t slaveAddress, uint8_t registerAddress, uint16_t data);
  static bool writeWords(uint8_t slaveAddress, uint8_t registerAddress, uint16_t* data, uint8_t length);

  static bool setTimeout(uint32_t timeout);

private:
  static uint32_t _timeout;
};