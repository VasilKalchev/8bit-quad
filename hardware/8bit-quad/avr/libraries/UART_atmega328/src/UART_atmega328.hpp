#pragma once
#include <stdint.h>

// #define P(string_literal) (reinterpret_cast<const __FlashStringHelper *>(PSTR(string_literal)))
// # define PSTR(s) ((const PROGMEM char *)(s))

namespace m328 {
namespace uart {

bool initialize(const uint32_t baud);

uint8_t read();
void print(char data);
void print(const char string[]);
void print(int8_t number, uint8_t radix = 10);
void print(int16_t number, uint8_t radix = 10);
void print(int32_t number, uint8_t radix = 10);
void print(uint8_t number, uint8_t radix = 10);
void print(uint16_t number, uint8_t radix = 10);
void print(uint32_t number, uint8_t radix = 10);
void print(float number, uint8_t precision = 2);

// void printChar(char data);
// void printString(const char string[]);
// void printUI(uint16_t number);
// void printFloat(float number);

// uint8_t read();
// void readString(char myString[], uint8_t maxLength);

} //namespace uart
} //namespace m328

namespace atmega328 = m328;
namespace atmega328p = m328;