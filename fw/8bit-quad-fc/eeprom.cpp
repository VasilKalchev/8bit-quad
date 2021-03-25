#include "eeprom.hpp"

/*
Macros
#define   EEMEM   __attribute__((section(".eeprom")))
#define   eeprom_is_ready()
#define   eeprom_busy_wait()   do {} while (!eeprom_is_ready())

Functions
uint8_t   eeprom_read_byte (const uint8_t *__p) __ATTR_PURE__
uint16_t  eeprom_read_word (const uint16_t *__p) __ATTR_PURE__
uint32_t  eeprom_read_dword (const uint32_t *__p) __ATTR_PURE__
float   eeprom_read_float (const float *__p) __ATTR_PURE__
void  eeprom_read_block (void *__dst, const void *__src, size_t __n)
void  eeprom_write_byte (uint8_t *__p, uint8_t __value)
void  eeprom_write_word (uint16_t *__p, uint16_t __value)
void  eeprom_write_dword (uint32_t *__p, uint32_t __value)
void  eeprom_write_float (float *__p, float __value)
void  eeprom_write_block (const void *__src, void *__dst, size_t __n)
void  eeprom_update_byte (uint8_t *__p, uint8_t __value)
void  eeprom_update_word (uint16_t *__p, uint16_t __value)
void  eeprom_update_dword (uint32_t *__p, uint32_t __value)
void  eeprom_update_float (float *__p, float __value)
void  eeprom_update_block (const void *__src, void *__dst, size_t __n)
*/

namespace eeprom {
bool read(const int16_t address, bool& value) {
  if (eeprom_is_ready()) {
    value = (bool)eeprom_read_byte((uint8_t*)address);
    return true;
  } else {
    return false;
  }
}
bool read(const int16_t address, int8_t& value) {
  if (eeprom_is_ready()) {
    value = (int8_t)eeprom_read_byte((uint8_t*)address);
    return true;
  } else {
    return false;
  }
}

bool read(const int16_t address, uint8_t& value) {
  if (eeprom_is_ready()) {
    value = (uint8_t)eeprom_read_byte((uint8_t*)address);
    return true;
  } else {
    return false;
  }
}

bool read(const int16_t address, int16_t& value) {
  if (eeprom_is_ready()) {
    value = (int16_t)eeprom_read_word((uint16_t*)address);
    return true;
  } else {
    return false;
  }
}

bool read(const int16_t address, uint16_t& value) {
  if (eeprom_is_ready()) {
    value = (uint16_t)eeprom_read_word((uint16_t*)address);
    return true;
  } else {
    return false;
  }
}

bool read(const int16_t address, int32_t& value) {
  if (eeprom_is_ready()) {
    value = (int32_t)eeprom_read_dword((uint32_t*)address);
    return true;
  } else {
    return false;
  }
}

bool read(const int16_t address, uint32_t& value) {
  if (eeprom_is_ready()) {
    value = (uint32_t)eeprom_read_dword((uint32_t*)address);
    return true;
  } else {
    return false;
  }
}

bool read(const int16_t address, float& value) {
  if (eeprom_is_ready()) {
    value = (float)eeprom_read_float((float*)address);
    return true;
  } else {
    return false;
  }
}


bool write(const int16_t address, const bool value) {
  if (eeprom_is_ready()) {
    eeprom_write_byte((uint8_t*)address, (uint8_t)value);
    return true;
  } else {
    return false;
  }
}

bool write(const int16_t address, const int8_t value) {
  if (eeprom_is_ready()) {
    eeprom_write_byte((uint8_t*)address, (uint8_t)value);
    return true;
  } else {
    return false;
  }
}

bool write(const int16_t address, const uint8_t value) {
  if (eeprom_is_ready()) {
    eeprom_write_byte((uint8_t*)address, (uint8_t)value);
    return true;
  } else {
    return false;
  }
}

bool write(const int16_t address, const int16_t value) {
  if (eeprom_is_ready()) {
    eeprom_write_word((uint16_t*)address, (uint16_t)value);
    return true;
  } else {
    return false;
  }
}

bool write(const int16_t address, const uint16_t value) {
  if (eeprom_is_ready()) {
    eeprom_write_word((uint16_t*)address, (uint16_t)value);
    return true;
  } else {
    return false;
  }
}

bool write(const int16_t address, const int32_t value) {
  if (eeprom_is_ready()) {
    eeprom_write_dword((uint32_t*)address, (uint32_t)value);
    return true;
  } else {
    return false;
  }
}

bool write(const int16_t address, const uint32_t value) {
  if (eeprom_is_ready()) {
    eeprom_write_dword((uint32_t*)address, (uint32_t)value);
    return true;
  } else {
    return false;
  }
}

bool write(const int16_t address, const float value) {
  if (eeprom_is_ready()) {
    eeprom_write_float((float*)address, (float)value);
    return true;
  } else {
    return false;
  }
}


bool update(const int16_t address, const bool value) {
  if (eeprom_is_ready()) {
    eeprom_update_byte((uint8_t*)address, (uint8_t)value);
    return true;
  } else {
    return false;
  }
}

bool update(const int16_t address, const int8_t value) {
  if (eeprom_is_ready()) {
    eeprom_update_byte((uint8_t*)address, (uint8_t)value);
    return true;
  } else {
    return false;
  }
}

bool update(const int16_t address, const uint8_t value) {
  if (eeprom_is_ready()) {
    eeprom_update_byte((uint8_t*)address, (uint8_t)value);
    return true;
  } else {
    return false;
  }
}

bool update(const int16_t address, const int16_t value) {
  if (eeprom_is_ready()) {
    eeprom_update_word((uint16_t*)address, (uint16_t)value);
    return true;
  } else {
    return false;
  }
}

bool update(const int16_t address, const uint16_t value) {
  if (eeprom_is_ready()) {
    eeprom_update_word((uint16_t*)address, (uint16_t)value);
    return true;
  } else {
    return false;
  }
}

bool update(const int16_t address, const int32_t value) {
  if (eeprom_is_ready()) {
    eeprom_update_dword((uint32_t*)address, (uint32_t)value);
    return true;
  } else {
    return false;
  }
}

bool update(const int16_t address, const uint32_t value) {
  if (eeprom_is_ready()) {
    eeprom_update_dword((uint32_t*)address, (uint32_t)value);
    return true;
  } else {
    return false;
  }
}

bool update(const int16_t address, const float value) {
  if (eeprom_is_ready()) {
    eeprom_update_float((float*)address, (float)value);
    return true;
  } else {
    return false;
  }
}
} //namespace eeprom