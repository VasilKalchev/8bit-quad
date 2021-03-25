#pragma once
#include <inttypes.h>

#include "IMU.hpp"


enum class Role : uint8_t {
  remote, drone,
};

enum class SettingId : uint8_t {
  dummy = 0,
  imuLpf_common, imuLpfAv_state, imuLpfAv_alpha,
  imuLpfAcc_state, imuLpfAcc_alpha,
  imuComplementary_alpha,
  comm_pa, comm_dataRate, comm_retryDelay, comm_retryCount,
  comm_crcLength, commTelemetry_type,
  regInner_p, regInner_yawP, regInner_outputLimit,
  regOuter_p, regOuter_i, regOuter_updateRate, regOuter_outputLimit,
  reg_minRegThrottle, reg_maxBaseThrottle,
  indication_armsLevel, indication_lamp,
  regAlt_p, regAlt_d,
};

enum class PacketType : uint8_t {
  Command = 0, Telemetry = 1, Telemetry_regulation = 2,
  Telemetry_imu = 3, Telemetry_motors = 4, Setting = 5,
};

enum class FlightMode : uint8_t {
  acro = 0, stabilize = 1, direct = 2,
};

struct Command {
  PacketType _type = PacketType::Command;
  uint32_t messageId = 0;
  int8_t senderId;
  int16_t throttle = 0;
  int16_t pitch = 0;
  int16_t roll = 0;
  int16_t yaw = 0;
  FlightMode flightMode = FlightMode::stabilize;
  bool altitudeHold = false;
};

struct Telemetry {
  PacketType _type;
  float altitude;
  float batteryVoltage;
};

struct Telemetry_regulation {
  PacketType _type;
  int16_t commandRoll;
  float avRoll;
  float attitudeRoll;
  float batteryVoltage;
};

struct Telemetry_imu {
  PacketType _type;
  AngularVelocity angularVelocity;
  Acceleration acceleration;
  Attitude attitude;
  float batteryVoltage;
};

struct Telemetry_motors {
  PacketType _type;
  uint8_t tl;
  uint8_t tr;
  uint8_t bl;
  uint8_t br;
  float batteryVoltage;
};

struct Setting {
  PacketType _type;
  SettingId id;
  float value;
  bool success;
  bool request;
};