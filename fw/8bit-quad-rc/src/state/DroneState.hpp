#pragma once

#include "../common/imu_types.hpp"


struct DroneSettings {
  DroneSettings() : innerP(-1.23), innerYawP(-1.23), outerP(-1.23), outerI(-1.23),
    avAlpha(-1.23), accAlpha(-1.23), commonAlpha(-123),
    armsLevel(-123), lampState(-123),
    telemetryType(-123) {}
  float innerP;
  float innerYawP;
  float outerP;
  float outerI;
  float avAlpha;
  float accAlpha;
  int8_t commonAlpha;
  int16_t armsLevel;
  int8_t lampState;
  int8_t telemetryType;
};

struct DroneStatus {
  DroneStatus() : batteryVoltage(0.0), altitude(0.0),
    tl(0), tr(0), bl(0), br(0) {
    angularVelocity.x = 0.0;
    angularVelocity.y = 0.0;
    angularVelocity.z = 0.0;
    acceleration.x = 0;
    acceleration.y = 0;
    acceleration.z = 0;
    attitude.pitch = 0.0;
    attitude.roll = 0.0;
  }
  float batteryVoltage;
  float altitude;
  AngularVelocity angularVelocity;
  Acceleration acceleration;
  Attitude attitude;
  uint8_t tl;
  uint8_t tr;
  uint8_t bl;
  uint8_t br;
};

enum class CurrentSetting : int8_t {
  START_OF_LIST = 0, innerP, innerYawP, outerP, outerI, armsLevel, commonAlpha, telemetryType, END_OF_LIST,
};

// Special behavior for ++CurrentSetting
CurrentSetting& operator++( CurrentSetting &c ) {
  c = static_cast<CurrentSetting>( static_cast<int8_t>(c) + 1 );
  if ( c == CurrentSetting::END_OF_LIST )
    c = static_cast<CurrentSetting>(static_cast<int8_t>(CurrentSetting::START_OF_LIST) + 1);
  return c;
}

// Special behavior for CurrentSetting++
CurrentSetting operator++( CurrentSetting &c, int ) {
  CurrentSetting result = c;
  ++c;
  return result;
}

// Special behavior for --CurrentSetting
CurrentSetting& operator--( CurrentSetting &c ) {
  c = static_cast<CurrentSetting>( static_cast<int8_t>(c) - 1 );
  if ( c == CurrentSetting::START_OF_LIST )
    c = static_cast<CurrentSetting>(static_cast<int8_t>(CurrentSetting::END_OF_LIST) - 1);
  return c;
}

// Special behavior for CurrentSetting--
CurrentSetting operator--( CurrentSetting &c, int ) {
  CurrentSetting result = c;
  --c;
  return result;
}