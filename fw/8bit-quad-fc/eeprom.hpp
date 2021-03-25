#pragma once
#include <avr/eeprom.h>


namespace eeprom {
bool read(const int16_t address, bool& value);
bool read(const int16_t address, int8_t& value);
bool read(const int16_t address, uint8_t& value);
bool read(const int16_t address, int16_t& value);
bool read(const int16_t address, uint16_t& value);
bool read(const int16_t address, int32_t& value);
bool read(const int16_t address, uint32_t& value);
bool read(const int16_t address, float& value);
bool write(const int16_t address, const bool value);
bool write(const int16_t address, const int8_t value);
bool write(const int16_t address, const uint8_t value);
bool write(const int16_t address, const int16_t value);
bool write(const int16_t address, const uint16_t value);
bool write(const int16_t address, const int32_t value);
bool write(const int16_t address, const uint32_t value);
bool write(const int16_t address, const float value);
bool update(const int16_t address, const bool value);
bool update(const int16_t address, const int8_t value);
bool update(const int16_t address, const uint8_t value);
bool update(const int16_t address, const int16_t value);
bool update(const int16_t address, const uint16_t value);
bool update(const int16_t address, const int32_t value);
bool update(const int16_t address, const uint32_t value);
bool update(const int16_t address, const float value);
} //namespace eeprom