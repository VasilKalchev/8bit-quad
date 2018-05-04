#pragma once

#include <EEPROM.h>
#include <Arduino.h>

#include "config.hpp"

enum class SettingId : uint8_t;

#define RESTORE_DEFAULTS true


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
    if (RESTORE_DEFAULTS) {
      EEPROM.put(_eepromAddress, _value);
      T val;
    } else {
      T value;
      EEPROM.get(_eepromAddress, value);
      _value = value;
    }
  }

  T getEepromValue() {
    T value;
    EEPROM.get(_eepromAddress, value);
    return value;
  }

  bool changeValue(T newValue) {
    if (_minimum >= _maximum || newValue < _minimum || newValue > _maximum) return false;
    EEPROM.put(_eepromAddress, newValue);
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
