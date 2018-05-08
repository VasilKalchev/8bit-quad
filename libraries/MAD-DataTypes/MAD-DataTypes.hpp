#pragma once
#include <stdint.h>

namespace mad {

namespace imu {
struct AngularVelocity_raw {
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
};
struct AngularVelocity {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Acceleration_raw {
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
};
struct Acceleration {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct MagneticField_raw {
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
};
struct MagneticField {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Attitude {
  float pitch = 0.0f;
  float roll = 0.0f;
  float yaw = 0.0f;
};
} //namespace imu


enum class ControllerType : int8_t {
  PID, PI_P,
};

enum class FlyMode : int8_t {
  acro = 0, // The remote's stick sets the angular velocity.
  angle = 1, // The remote's stick sets the angle.
  horizon = 2, // If remote's stick is more centered - 'Angle', if not - 'Acro'.
  land = 3, // Automonous landing.
  direct = 4, // The remote's stick sets the difference in motor power.
  first = acro,
  last = direct,
  noConnection,
};

struct Control {
  float throttle = 0.0f;
  float pitch = 0.0f;
  float pitchAngle = 0.0f;
  float pitchVelocity = 0.0f;
  float roll = 0.0f;
  float rollAngle = 0.0f;
  float rollVelocity = 0.0f;
  float yawVelocity = 0.0f;
  FlyMode flyMode = FlyMode::Angle;
  bool holdAltitude = false;
};


namespace communication {
enum class MessageType : int8_t {
  command = -2, setting = -1, telemetryNormal = 0,
};

struct Command {
  const MessageType _messageType = MessageType::command;
  int16_t throttle_c = 0; // in centi (divide by 100)
  int16_t pitch_c = 0; // in centidegrees
  int16_t roll_c = 0; // in centidegrees
  int16_t yaw_c = 0; // in centidegrees
  FlyMode flyMode = FlyMode::Angle;
  bool holdAltitude = false;
  int8_t remoteId = 0;
};

enum class SettingId : int8_t {
  invalid = -1,
  innerP, innerYawP, innerYawI, outerP, outerI,
  p, i, d, yawP, yawI,
  altitudeInnerP, altitudeOuterP, altitudeOuterI,
  altitudeP, altitudeI, altitudeD,
  armsLevel, lampState,
  telemetryType,
  calibrateGyroscope, calibrateAccelerometer,
  land,
};
struct Setting {
  const MessageType _messageType = MessageType::setting;
  SettingId id = SettingId::invalid;
  bool request = true;
  bool success = false;
  float value = 0.0f;
  int8_t remoteId = 0;
};

struct TelemetryNormal {
  const MessageType _messageType = MessageType::telemetryNormal;
  uint16_t batteryVoltage_m = 0; // in millivolts
  int16_t altitude_c = 0; // in centimeters
};
} //namespace communication

} //namespace mad