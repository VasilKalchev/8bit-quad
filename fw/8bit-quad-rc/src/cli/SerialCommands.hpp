#pragma once
#include <inttypes.h>
#include <stdlib.h>
#include <Arduino.h>

namespace sercom {

uint32_t commandToCode(const char * const command);
constexpr uint32_t stringToCode(const char * const string) {
  return (0 | ((uint32_t)string[0] << 0) |
          ((uint32_t)string[1] << 8) | ((uint32_t)string[2] << 16));
}
int32_t commandToLong(const char * const command);
int16_t commandToInt(const char * const command);
float commandToFloat(const char * const command);
bool isValid(const char * const command);
void commandToValueString(const char * const command, char * const valueString);

} //namespace sercom
