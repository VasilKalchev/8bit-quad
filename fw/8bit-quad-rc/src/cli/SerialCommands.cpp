#include "SerialCommands.hpp"

namespace sercom {


uint32_t commandToCode(const char * const command) {
  return (0 | (uint32_t)command[2] << 0 |
          ((uint32_t)command[3] << 8) | ((uint32_t)command[4] << 16));
}

// uint32_t stringToCode(const char * const string) {
//   return (0 | ((uint32_t)string[0] << 0) |
//           ((uint32_t)string[1] << 8) | ((uint32_t)string[2] << 16));
// }

int32_t commandToLong(const char * const command) {
  char valueString[7] = { 32, 32, 32, 32, 32, 32, 32 };
  commandToValueString(command, valueString);
  return atol(valueString);
}

int16_t commandToInt(const char * const command) {
  char valueString[7] = { 32, 32, 32, 32, 32, 32, 32 };
  commandToValueString(command, valueString);
  return atoi(valueString);
}

float commandToFloat(const char * const command) {
  char valueString[7] = { 32, 32, 32, 32, 32, 32, 32 };
  commandToValueString(command, valueString);
  return atof(valueString);
}

bool isValid(const char * const command) {
  if (command[0] == 's'
      && (command[1] != '.' || command[5] != '-'
          || (command[7] != ';' && command[8] != ';'
              && command[9] != ';' && command[10] != ';'
              && command[11] != ';' && command[12] != ';') ) ) {
    return false;
  } else if (command[0] == 'g' && command[5] != ';') {
    return false;
  } else if (command[0] != 'g' && command[0] != 's') {
    return false;
  } else if (command[0] == 's') {
    int8_t digits = 0;
    for (int8_t i = 7; i < 12; ++i) {
      if (command[i] == ';') digits = i - 6;
    }
    if (digits == 0) return false;
    else return true;
  } else {
    return true;
  }
}

void commandToValueString(const char * const command, char * const valueString) {
  int8_t digits = 0;
  for (int8_t i = 7; i < 12; ++i) {
    if (command[i] == ';') digits = i - 6;
  }
  int8_t c = 0;
  for (int8_t i = 7; i > 7 - digits; --i) {
    valueString[i - 1] = command[5 + digits - c];
    ++c;
  }
}


} //namespace sercom