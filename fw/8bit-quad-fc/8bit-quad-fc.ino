#include "src/config/config.hpp"
#include "src/util/debug.hpp"
#include "src/board/8bit-quad-fc_board.hpp"
#include "src/peripheral/ADC.hpp"
#include "UART_atmega328.hpp"
#include "src/imu/MPU9255.hpp"
#include "src/control/PID.hpp"
#include "src/radio/nRF24L01p.hpp"
#include "src/peripheral/eeprom.hpp"
#include "src/util/utils.hpp"

#include <MahonyAHRS.hpp>
Mahony mahony(1.0f / 322.0f); // sample period, seconds (322 Hz)

#include <i2c_BMP280.h>
BMP280 bmp;
float takeOffAltitude = 0.0f;
float takeOffPressure = 0.0f;
float relativeAltitude = 0.0f;
const float seaLevelPressure = 101325.0f;

using namespace m328;

namespace imu {
MPU9255 mpu9255(config::imu::i2cAddress);
} //namespace imu
AngularVelocity angularVelocity;
Acceleration acceleration;
Attitude attitude;
float avPitch = 0.0f;
float avRoll = 0.0f;
float avYaw = 0.0;


namespace comm {
nRF24L01p_RF24 rf24(Role::drone, pin::communication::ino::ce, pin::communication::ino::csn);
} //namespace comm
Command command;
int16_t throttle = 0;

namespace ctlr {

namespace att {

namespace rate {
namespace setpoint {
int16_t pitch = 0;
int16_t roll = 0;
} //namespace setpoint
namespace output {
int16_t pitch = 0;
int16_t roll = 0;
int16_t yaw = 0;
} //namespace output
Pcontroller pitch(&setpoint::pitch, &angularVelocity.x, &output::pitch);
Pcontroller roll(&setpoint::roll, &angularVelocity.y, &output::roll);
PID yaw(&command.yaw, &avYaw, &output::yaw);
} //namespace rate

namespace angle {
PID pitch(&command.pitch, &attitude.pitch, &rate::setpoint::pitch);
PID roll(&command.roll, &attitude.roll, &rate::setpoint::roll);
} //namespace angle

} //namespace att

} //namespace ctlr


namespace status {
Status communication = Status::normal;
Status battery = Status::normal;
} //namespace status


float batteryVoltage = 0.0;



void handleCommand();
void handleSetting(Setting & setting);

#define EXACT_TIMING true
#if EXACT_TIMING
#  define WAIT_FOR_CYCLE(LUS) while (micros() - LUS < config::cycleTime);
#else
#  define WAIT_FOR_CYCLE(LUS)
#endif


void setup() {
  bool success = true;

  replaceTimer0WithTimer2();

  initIO();
  indication::signal(OFF);
  indication::warning(ON);
  indication::arms(OFF);

  Wire.begin();
  Wire.setClock(800000); // was 400000

  INIT_UART(config::debug::baud);
  uart::initialize(config::debug::baud);
  DEBUGLN("FC2.\n");

  config::init();
#if DEBUG_SETTINGS
  DEBUGLN("  SETTINGS");
  DEBUG("  Telemetry type: "); DEBUGLN(config::communication::telemetry::type());
  DEBUG("  Pi: "); DEBUGLN(config::ctlr::att::rate::P());
  DEBUG("  Pi_yaw: "); DEBUGLN(config::ctlr::att::rate::yawP());
  DEBUG("  Po: "); DEBUGLN(config::ctlr::att::angle::P());
  DEBUG("  Io: "); DEBUGLN(config::ctlr::att::angle::I());
  DEBUG("  Arms level (0-12): "); DEBUGLN(config::indication::armsLevel());
  DEBUG("  Lamp: "); if (config::indication::lamp()) {DEBUGLN("ON");} else {DEBUGLN("OFF");}
  DEBUGLN("  -----\n");
#endif

  initADC();

  // IMU initialization
  DEBUG("IMU");
  success &= imu::mpu9255.initialize();
  DEBUG(" done. T = "); DEBUG(imu::mpu9255.getTemperature()); DEBUGLN("degC.");
  // ~IMU initialization -----


  // Communication initialization
  DEBUG("Communication");
  comm::rf24.initialize();
  comm::rf24.setPALevel(RF24_PA_MAX);
  //RF24_PA_MIN = 0,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR - rf24_pa_dbm_e;
  comm::rf24.setDataRate(RF24_2MBPS);
  //RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS - rf24_datarate_e;
  comm::rf24.setRetries(config::communication::retryDelay, config::communication::retryCount);
  comm::rf24.setCRCLength(config::communication::crcLength);
  success &= comm::rf24.isChipConnected();
  if (comm::rf24.isChipConnected()) { DEBUGLN(" done."); }
  // ~Communication initialization -----

  // Regulation initialization
  // DEBUG("Regulation");
  success &= ctlr::att::rate::pitch.setTunings(config::ctlr::att::rate::P());
  success &= ctlr::att::rate::roll.setTunings(config::ctlr::att::rate::P());
  success &= ctlr::att::rate::yaw.setTunings(config::ctlr::att::rate::yawP(),
                                        config::ctlr::att::rate::yawI);
  ctlr::att::rate::yaw.setUpdateRate(config::ctlr::att::rate::yaw_updateRate);
  success &= ctlr::att::rate::pitch.setOutputLimits(config::ctlr::att::rate::outputLimit);
  success &= ctlr::att::rate::roll.setOutputLimits(config::ctlr::att::rate::outputLimit);
  success &= ctlr::att::rate::yaw.setOutputLimits(config::ctlr::att::rate::outputLimit);
  success &= ctlr::att::rate::yaw.setIntegralLimit(2);
  ctlr::att::rate::pitch.on(); ctlr::att::rate::roll.on(); ctlr::att::rate::yaw.on();

  success &= ctlr::att::angle::pitch.setTunings(config::ctlr::att::angle::P(), config::ctlr::att::angle::I());
  success &= ctlr::att::angle::roll.setTunings(config::ctlr::att::angle::P(), config::ctlr::att::angle::I());
  ctlr::att::angle::pitch.setUpdateRate(config::ctlr::att::angle::updateRate);
  ctlr::att::angle::roll.setUpdateRate(config::ctlr::att::angle::updateRate);
  success &= ctlr::att::angle::pitch.setOutputLimits(config::ctlr::att::angle::outputLimit);
  success &= ctlr::att::angle::roll.setOutputLimits(config::ctlr::att::angle::outputLimit);
  success &= ctlr::att::angle::pitch.setIntegralLimit(64);
  success &= ctlr::att::angle::roll.setIntegralLimit(64);
  ctlr::att::angle::pitch.on(); ctlr::att::angle::roll.on();
  // ~Regulation initialization -----


  bmp.initialize();
  bmp.setPressureOversampleRatio(16);
  bmp.setTemperatureOversampleRatio(1);
  bmp.setFilterRatio(16);
  bmp.setStandby(0);
  bmp.setEnabled(true);

  float p = 0.0f;
  float t = 0.0f;
  for (uint16_t i = 0; i < 100; ++i) {
    bmp.read(p, t);
  }
  float pSum = 0.0f;
  float tSum = 0.0f;
  const uint16_t samples = 10;
  for (uint16_t i = 0; i < samples; ++i) {
    bmp.read(p, t);
    pSum += p;
    tSum += t;
  }
  p = pSum / samples;
  t = tSum / samples;

  takeOffPressure = p;
  takeOffAltitude = calculateAltitude(p, seaLevelPressure, t);
  uart::print("P: "); uart::print(p);
  uart::print("Pa, A: "); uart::print(takeOffAltitude); uart::print("m\n");


  if (!success) { DEBUG("Wrong initialization configuration! Terminated."); exit(0); }
  indication::signal(ON);
  indication::warning(OFF);
  indication::lamp(config::indication::lamp());
  // indication::arms(0);
  // DEBUGLN("Setup done.\n----------");
} //void setup()


void loop() {
  static uint32_t microsThisCycle = micros();

#if DEBUG_LOOP_TIME
  uart::print(micros() - microsThisCycle);
#endif

  microsThisCycle = micros();


  // Regulation
  static uint32_t innerComputeLUS = microsThisCycle;
  WAIT_FOR_CYCLE(innerComputeLUS);
  innerComputeLUS = micros();
  ctlr::att::rate::yaw.compute();
  ctlr::att::rate::pitch.compute();
  ctlr::att::rate::roll.compute();

  /* Cycles:
  0 - Compute outer PID
  1 - Battery voltage, Altitude
  2 - Communication[1/2], Indication[1/2]
  3 - Communication[1/2], Indication[1/2]
  */
  static int8_t innerCycles = 0;
  ++innerCycles;

  if (innerCycles >= 4 || innerCycles < 0) {
    innerCycles = 0;
    ctlr::att::angle::pitch.compute();
    ctlr::att::angle::roll.compute();
  }

#if DEBUG_PID_PITCH
  if (!innerCycles % 3) {
    if (command.flightMode == FlightMode::stabilize) {
      uart::print(command.pitch);
      uart::print(","); uart::print(attitude.pitch);
    } else {
      uart::print(","); uart::print(ctlr::att::rate::setpoint::pitch);
      uart::print(","); uart::print(angularVelocity.y);
    }
    uart::print(","); uart::print(ctlr::att::rate::output::pitch); uart::print("\n");
  }
#endif
#if DEBUG_PID_ROLL
  if (!innerCycles % 3) {
    if (command.flightMode == FlightMode::stabilize) {
      uart::print(command.roll);
      uart::print(","); uart::print(attitude.roll);
    }
    uart::print(","); uart::print(ctlr::att::rate::setpoint::roll);
    uart::print(","); uart::print(angularVelocity.x);
    uart::print(","); uart::print(ctlr::att::rate::output::roll); uart::print("\n");
  }
#endif
#if DEBUG_PID_YAW
  if (!innerCycles % 3) {
    uart::print(command.yaw);
    uart::print(", "); uart::print(avYaw);
    uart::print(", "); uart::print(ctlr::att::rate::output::yaw);
    uart::print("\n");
  }
#endif
  // ~Regulation -----


  // Attitude -----
  // Fetch
  imu::mpu9255.getMotion(&angularVelocity, &acceleration);


#if DEBUG_ANGULAR_VELOCITY
  if (innerCycles == 0) {
    uart::print(angularVelocity.x); uart::print(", ");
    uart::print(angularVelocity.y); uart::print(", ");
    uart::print(angularVelocity.z); uart::print("\n");
  }
#endif
#if DEBUG_ACCELERATION
  if (innerCycles == 0) {
    uart::print(acceleration.x); uart::print(", ");
    uart::print(acceleration.y); uart::print(", ");
    uart::print(acceleration.z); uart::print("\n");
  }
#endif


#define MAHONY_FUSION true
#if MAHONY_FUSION
  static uint32_t mahonyLUS = micros();
  WAIT_FOR_CYCLE(mahonyLUS);
  mahonyLUS = micros();
  float mahonyYaw, mahonyPitch, mahonyRoll;
  mahony.update(mahonyYaw, mahonyPitch, mahonyRoll,
                acceleration.x, acceleration.y, acceleration.z,
                angularVelocity.x * DEG_TO_RAD, angularVelocity.y * DEG_TO_RAD, angularVelocity.z * DEG_TO_RAD);
  attitude.roll = mahonyPitch * RAD_TO_DEG;
  attitude.pitch = mahonyRoll * RAD_TO_DEG;
  float mYaw = mahonyYaw * RAD_TO_DEG;

  static uint32_t dVelocityLUS = microsThisCycle;
  static float attitudeYawLast = mYaw;
  uint32_t us = micros();
  uint32_t vDt = (us - dVelocityLUS + 4) / 1000000.0f;
  avYaw = (mYaw - attitudeYawLast) / vDt;
  attitudeYawLast = mYaw;
  dVelocityLUS = us;
#endif


#if DEBUG_ATTITUDE
  if (innerCycles == 0) {
    uart::print(attitude.pitch); uart::print(", "); uart::print(attitude.roll); uart::print("\n");
  }
#endif
  // ~Attitude -----


  // Motor mix
  uint8_t tl_p = 0;
  uint8_t tr_p = 0;
  uint8_t bl_p = 0;
  uint8_t br_p = 0;
  if (throttle > config::ctlr::minimumRegulationThrottle) {
    if (throttle > config::ctlr::maximumBaseThrottle) throttle = config::ctlr::maximumBaseThrottle;
    tl_p = clamp(throttle
                 + ctlr::att::rate::output::pitch + ctlr::att::rate::output::roll - ctlr::att::rate::output::yaw,
                 0, 127);
    tr_p = clamp(throttle
                 + ctlr::att::rate::output::pitch - ctlr::att::rate::output::roll + ctlr::att::rate::output::yaw,
                 0, 127);
    bl_p = clamp(throttle
                 - ctlr::att::rate::output::pitch + ctlr::att::rate::output::roll + ctlr::att::rate::output::yaw,
                 0, 127);
    br_p = clamp(throttle
                 - ctlr::att::rate::output::pitch - ctlr::att::rate::output::roll - ctlr::att::rate::output::yaw,
                 0, 127);
  } else {
    tl_p = throttle;
    tr_p = throttle;
    bl_p = throttle;
    br_p = throttle;
    ctlr::att::angle::pitch.unwind();
    ctlr::att::angle::roll.unwind();
    ctlr::att::rate::yaw.unwind();
  }
  OCR0B = 127 + tl_p;
  OCR1B = 127 + tr_p;
  OCR0A = 127 + bl_p;
  OCR1A = 127 + br_p;

#if DEBUG_MOTORS
  if (innerCycles == 0) {
    static uint32_t dm_lm = millis();
    if (millis() > dm_lm + 50) {
      dm_lm = millis();
      uart::print(OCR0B); uart::print("\t"); uart::print(OCR1B); uart::print("\n");
      uart::print(OCR0A); uart::print("\t"); uart::print(OCR1A); uart::print("\n");
      uint8_t i = 0;
      while (i++ < 12) uart::print("\n");
    }
  }
#endif
// ~Motor mix -----


// Communication
  static uint32_t lastCommandLUS = microsThisCycle;
  static uint32_t batteryVoltageLUS = microsThisCycle;
  if (innerCycles == 1 && comm::rf24.available()) {
    while (comm::rf24.available()) {
      uint8_t rawMessage[32];
      comm::rf24.read(&rawMessage, 32);
      switch ((PacketType)rawMessage[0]) {
      case PacketType::Command:
        memcpy(&command, &rawMessage, sizeof(command));
        handleCommand();
        lastCommandLUS = microsThisCycle;
        break;
      case PacketType::Setting:
        Setting setting;
        memcpy(&setting, &rawMessage, sizeof(setting));
        handleSetting(setting);
        break;
      default:
        comm::rf24.reset();
        DEBUG("unrec "); DEBUGLN(rawMessage[0]);
        break;
      }
    }
  } else if (microsThisCycle - lastCommandLUS > config::communication::commandTimeout) {
    throttle = 0;
    ctlr::att::angle::pitch.on();
    ctlr::att::angle::roll.on();
    command.pitch = 0;
    command.roll = 0;
    command.yaw = 0;
    status::communication = Status::error;
#if DEBUG_COMMAND
    if (command.flightMode == FlightMode::stabilize) DEBUG("STAB - ");
    else if (command.flightMode == FlightMode::acro) DEBUG("ACRO - ");
    else DEBUG("DIRECT - ");
    DEBUG("TH: "); DEBUG(command.throttle); DEBUG(", P: "); DEBUG(command.pitch);
    DEBUG(", R: "); DEBUG(command.roll); DEBUG(", Y: "); DEBUG(command.yaw);
    DEBUGLN();
#endif
  } else if (innerCycles == 3 && (microsThisCycle - batteryVoltageLUS > config::battery::updateRate)) {
    batteryVoltageLUS = microsThisCycle;
    batteryVoltage = readBatteryVoltage();
    static float batteryVoltage_last = 0.0;
    batteryVoltage = lowPassFilter(batteryVoltage, batteryVoltage_last, config::battery::alpha);
    batteryVoltage_last = batteryVoltage;
    if (batteryVoltage > config::battery::lowVoltage) {
      status::battery = Status::normal;
    } else if (batteryVoltage > config::battery::criticalVoltage) {
      status::battery = Status::warning;
    } else {
      status::battery = Status::error;
    }
  }
// ~Communication -----


  // Pressure/altitude (telemetry only, no altitude hold) -----
  static float pressure = takeOffPressure;
  static float pressurePrev = takeOffPressure;
  static float temperature = 0.0;
  static float pressureRaw[3] = { takeOffPressure, takeOffPressure, takeOffPressure };
  static uint8_t pressureNdx = 0;

  if (innerCycles == 2) { //~700us
    bmp.read(pressureRaw[pressureNdx], temperature);
  } else if (innerCycles == 3) { //~400us
    if (pressureNdx >= 2) {
      pressureNdx = 0;
      pressure = middle_of_3(pressureRaw[0], pressureRaw[1], pressureRaw[2]);
    } else {
      ++pressureNdx;
    }

    if (pressureNdx == 0) {
      pressure = pressure * 0.07f + pressurePrev * 0.93f;
      pressurePrev = pressure;
      // Serial.print("P: "); Serial.print(pressure); Serial.print("hPa\t");

      float absoluteAltitude = calculateAltitude(pressure, seaLevelPressure, temperature);
      relativeAltitude = absoluteAltitude - takeOffAltitude;

      static float relativeAltitudeLastFilter = relativeAltitude;
      relativeAltitude = relativeAltitude * 0.08f + relativeAltitudeLastFilter * 0.92f;
      relativeAltitudeLastFilter = relativeAltitude;
      // Serial.print("A: "); Serial.print(absoluteAltitude); Serial.print("m\t");
      // Serial.print("RA: "); Serial.print(relativeAltitude); Serial.print("m\t");

      if (absoluteAltitude < takeOffAltitude - 1.5f) {
        takeOffAltitude = absoluteAltitude;
      }
    }
  }
  // ~Pressure/altitude -----


// Indication
  if ((innerCycles == 1 || innerCycles == 3) && status::communication == Status::normal) {
    if (indication::arms() != expMap[clamp(config::indication::armsLevel(), 0, 13)]) {
      uint8_t armsPwm = indication::arms();
      if (armsPwm > expMap[clamp(config::indication::armsLevel(), 0, 13)]) {
        indication::arms(--armsPwm);
      } else if (armsPwm < expMap[clamp(config::indication::armsLevel(), 0, 13)]) {
        indication::arms(++armsPwm);
      }
    }
  }
  static uint32_t indicationLUS = microsThisCycle;
  if ((innerCycles == 1) && microsThisCycle - indicationLUS > config::indication::period) {
    indicationLUS = microsThisCycle;
    if (status::battery == Status::normal && status::communication == Status::normal) {
      indication::toggleSignal();
      indication::warning(0);
    } else if (status::communication != Status::normal) {
      indication::toggleSignal();
      indication::warning(!indication::signal());
      indication::toggleArms();
    } else if (status::battery == Status::warning) {
      indication::toggleSignal();
      indication::warning(!indication::signal());
    } else if (status::battery == Status::error) {
      indication::signal(0);
      indication::toggleWarning();
    } else {
      status::communication = Status::normal;
      status::battery = Status::normal;
    }
  }
// ~Indication -----


} //void loop()


void handleCommand() {
  switch (config::communication::telemetry::type()) {
  case 0:
    comm::rf24.setResponse(nullptr, sizeof(nullptr));
    break;
  case 1:
    static Telemetry telemetry;
    telemetry._type = PacketType::Telemetry;
    telemetry.batteryVoltage = batteryVoltage;
    telemetry.altitude = relativeAltitude;
    comm::rf24.setResponse(&telemetry, sizeof(telemetry));
    break;
  case 2:
    static Telemetry_regulation telemetry_regulation;
    telemetry_regulation._type = PacketType::Telemetry_regulation;
    telemetry_regulation.batteryVoltage = batteryVoltage;
    telemetry_regulation.commandRoll = command.roll;
    telemetry_regulation.avRoll = angularVelocity.y;
    telemetry_regulation.attitudeRoll = attitude.roll;
    comm::rf24.setResponse(&telemetry_regulation, sizeof(telemetry_regulation));
    break;
  case 3:
    static Telemetry_imu telemetry_imu;
    telemetry_imu._type = PacketType::Telemetry_imu;
    telemetry_imu.angularVelocity.x = angularVelocity.x;
    telemetry_imu.angularVelocity.y = angularVelocity.y;
    telemetry_imu.angularVelocity.z = angularVelocity.z;
    telemetry_imu.acceleration.x = acceleration.x;
    telemetry_imu.acceleration.y = acceleration.y;
    telemetry_imu.acceleration.z = acceleration.z;
    telemetry_imu.attitude.pitch = attitude.pitch;
    telemetry_imu.attitude.roll = attitude.roll;
    telemetry_imu.batteryVoltage = batteryVoltage;
    comm::rf24.setResponse(&telemetry_imu, sizeof(telemetry_imu));
    break;
  case 4:
    static Telemetry_motors telemetry_motors;
    telemetry_motors._type = PacketType::Telemetry_motors;
    telemetry_motors.tl = OCR0B;
    telemetry_motors.tr = OCR1B;
    telemetry_motors.bl = OCR0A;
    telemetry_motors.br = OCR1A;
    telemetry_motors.batteryVoltage = batteryVoltage;
    comm::rf24.setResponse(&telemetry_motors, sizeof(telemetry_motors));
    break;
  default: break;
  }

  static uint8_t senderId = command.senderId;
  if (command.senderId != senderId
      || command.throttle < 0 || command.throttle > 127
      || command.pitch < -200 || command.pitch > 200
      || command.roll < -200 || command.roll > 200) {
    OCR0B = 0;
    OCR1B = 0;
    OCR0A = 0;
    OCR1A = 0;
    throttle = 0;
    command.pitch = 0;
    command.roll = 0;
    command.yaw = 0;
    command.flightMode = FlightMode::stabilize;
  } else {
    if (command.flightMode == FlightMode::stabilize) {
      ctlr::att::angle::pitch.setMode(AUTOMATIC);
      ctlr::att::angle::roll.setMode(AUTOMATIC);
    } else if (command.flightMode == FlightMode::acro) {
      ctlr::att::angle::pitch.setMode(MANUAL);
      ctlr::att::angle::roll.setMode(MANUAL);
      ctlr::att::rate::setpoint::pitch = command.pitch;
      ctlr::att::rate::setpoint::roll = command.roll;
    } else {
      ctlr::att::rate::pitch.off();
      ctlr::att::rate::roll.off();
      ctlr::att::angle::pitch.off();
      ctlr::att::angle::roll.off();
    }
    throttle = command.throttle;
    status::communication = Status::normal;
  }
}

void handleSetting(Setting & setting) {
  setting.success = true;
  switch (setting.id) {
  case SettingId::dummy:
    setting.success = false;
    DEBUGLN("dummy");
    break;

  case SettingId::commTelemetry_type:
    if (setting.request) {
      DEBUGLN("?Tmt");
      setting.value = (float)config::communication::telemetry::type();
    } else {
      setting.success = config::communication::telemetry::type.changeValue(setting.value);
      setting.value = (float)config::communication::telemetry::type();
      DEBUG("Tmt>"); DEBUGLN(setting.value);
    }
    break;

  case SettingId::regInner_p:
    if (setting.request) {
      DEBUGLN("?Pi");
      setting.value = config::ctlr::att::rate::P();
    } else {
      setting.success = config::ctlr::att::rate::P.changeValue(setting.value);
      setting.value = config::ctlr::att::rate::P();
      ctlr::att::rate::pitch.setTunings(setting.value);
      ctlr::att::rate::roll.setTunings(setting.value);
      DEBUG("Pi>"); DEBUGLN(setting.value);
    }
    break;
  case SettingId::regInner_yawP:
    if (setting.request) {
      DEBUGLN("?Pi_yaw");
      setting.value = config::ctlr::att::rate::yawP();
    } else {
      setting.success = config::ctlr::att::rate::yawP.changeValue(setting.value);
      setting.value = config::ctlr::att::rate::yawP();
      ctlr::att::rate::yaw.setTunings(setting.value);
      DEBUG("Pi_yaw>"); DEBUGLN(setting.value);
    }
    break;
  case SettingId::regOuter_p:
    if (setting.request) {
      DEBUGLN("?Po");
      setting.value = config::ctlr::att::angle::P();
    } else {
      setting.success = config::ctlr::att::angle::P.changeValue(setting.value);
      setting.value = config::ctlr::att::angle::P();
      ctlr::att::angle::pitch.setTunings(setting.value, config::ctlr::att::angle::I());
      ctlr::att::angle::roll.setTunings(setting.value, config::ctlr::att::angle::I());
      DEBUG("Po>"); DEBUGLN(setting.value);
    }
    break;
  case SettingId::regOuter_i:
    if (setting.request) {
      DEBUGLN("?Io");
      setting.value = config::ctlr::att::angle::I();
    } else {
      setting.success = config::ctlr::att::angle::I.changeValue(setting.value);
      setting.value = config::ctlr::att::angle::I();
      ctlr::att::angle::pitch.setTunings(config::ctlr::att::angle::P(), setting.value);
      ctlr::att::angle::roll.setTunings(config::ctlr::att::angle::P(), setting.value);
      DEBUG("Io>"); DEBUGLN(setting.value);
    }
    break;

  case SettingId::indication_armsLevel:
    if (setting.request) {
      DEBUGLN("?Arms lvl");
      setting.value = (float)config::indication::armsLevel();
    } else {
      setting.success = config::indication::armsLevel.changeValue(setting.value);
      setting.value = (float)config::indication::armsLevel();
      DEBUG("Arms lvl>"); DEBUGLN(setting.value);
    }
    break;
  case SettingId::indication_lamp:
    if (setting.request) {
      DEBUGLN("?Lamp");
      setting.value = (float)config::indication::lamp();
    } else {
      setting.success = config::indication::lamp.changeValue(setting.value);
      setting.value = (float)config::indication::lamp();
      indication::lamp(config::indication::lamp());
      DEBUG("Lamp>"); DEBUGLN(setting.value);
    }
    break;

  default:
    setting.success = false;
    DEBUG("Unrec sId: "); DEBUGLN((uint8_t)setting.id);
    break;
  } //switch (setting.id)
  comm::rf24.setResponse(&setting, sizeof(setting));
}
