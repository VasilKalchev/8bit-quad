#pragma once

// #include <avr/eeprom.h>
#include "eeprom.hpp"
#include <Arduino.h>

#include "config.hpp"

enum class SettingId : uint8_t;

#define RESTORE_DEFAULTS false


  template <class T>
  class EepromSetting {
public:
  EepromSetting(SettingId id, T value, int16_t eepromAddress, T minimum = 0, T maximum = 0, T step = 1) {
    _value = value;
    _id = id;
    _eepromAddress = eepromAddress;
    _minimum = minimum;
    _maximum = maximum;
    _step = step;
  }

  void init() {
    // Serial.print((uint8_t)_id);
    if (RESTORE_DEFAULTS) {
      while (!eeprom_is_ready());
      eeprom::update(_eepromAddress, _value);
      // Serial.print(": restoring the default of ");
    } else {
      T value = 0;
      while (!eeprom_is_ready());
      eeprom::read(_eepromAddress, value);
      _value = value;
      // Serial.print(": reading from eeprom ");
    }
    // Serial.println(_value);
  }

  T getEepromValue() {
    T value;
    while (!eeprom_is_ready());
    eeprom::read(_eepromAddress, value);
    return value;
  }

  bool changeValue(T newValue) {
    if (_minimum >= _maximum || newValue < _minimum || newValue > _maximum) return false;
    while (!eeprom_is_ready());
    eeprom::update(_eepromAddress, newValue);
    _value = newValue;
    return true;
  }

  T operator()() {
    return _value;
  }

  T getMinimum() { return _minimum; }
  T getMaximum() { return _maximum; }
  T getStep() { return _step; }

private:
  SettingId _id;
  int16_t _eepromAddress;
  T _minimum;
  T _maximum;
  T _step;
  T _value;
};
