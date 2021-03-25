#pragma once
#include <inttypes.h>

#include <SPI.h>
#include "RF24.h"
#include "Communication.hpp"

// const uint64_t _pipe[5] = { 0x3A3A3A3AD2LL, 0x3A3A3A3AC3LL, 0x3A3A3A3AB4LL, 0x3A3A3A3AA5LL, 0x3A3A3A3A96LL };
const uint64_t _pipe[] = { 0x3A3A3A3AD2LL, 0x3A3A3A3AC3LL };

class nRF24L01p_RF24 {
public:
  nRF24L01p_RF24(Role role, uint8_t ce = 7, uint8_t csn = 8);

  void initialize();
  void setPALevel(uint8_t paLevel);
  void setDataRate(uint8_t dataRate);
  void setRetries(uint8_t delay, uint8_t count);
  void setCRCLength(uint8_t crcLength);
  void reset();

  bool available();
  void startListening();
  void stopListening();
  bool write(const void *buffer, uint8_t length);
  void read(void *buffer, uint8_t length);
  void deserialize(void *dst, const void *src, uint8_t length);
  void setResponse(const void* buffer, uint8_t length);

  bool isChipConnected();

private:
  RF24 _rf24;
  Role _role;
};
