#include <avr/io.h>
#include <util/delay.h>
#include <inttypes.h>

#include "src/config/config.hpp"
#include "src/util/debug.hpp"
#include "src/board/8bit-quad-rc_board.hpp"
#include "src/peripheral/UART.hpp"
#include "src/peripheral/ADC.hpp"
#include "src/radio/nRF24L01p.hpp"
#include "src/state/DroneState.hpp"
#include "src/cli/SerialCommands.hpp"
#include "src/util/utils.hpp"


struct InputState {
  InputState() : throttle(0), pitch(0), roll(0),
    yawControl(false), flightMode(FlightMode::stabilize), menu(false), fn(false), sw(false) {}
  int16_t throttle;
  int16_t pitch;
  int16_t roll;
  bool yawControl;
  FlightMode flightMode;
  bool menu;
  bool fn;
  bool sw;
} inputState;

DroneSettings droneSettings;
DroneStatus droneStatus;

namespace comm {
nRF24L01p_RF24 rf24(Role::remote, pin::communication::ino::ce, pin::communication::ino::csn);
} //namespace comm


namespace status {
Status communication = Status::normal;
Status battery = Status::normal;
} //namespace status


int16_t pitchStickCenter = 512;
int16_t rollStickCenter = 512;
float batteryVoltage = 12.6;
bool directFlightMode = false;

constexpr int32_t irp = sercom::stringToCode("irp");
constexpr int32_t iry = sercom::stringToCode("iry");
constexpr int32_t orp = sercom::stringToCode("orp");
constexpr int32_t ori = sercom::stringToCode("ori");
constexpr int32_t clf = sercom::stringToCode("clf");
constexpr int32_t glf = sercom::stringToCode("glf");
constexpr int32_t alf = sercom::stringToCode("alf");
constexpr int32_t fcf = sercom::stringToCode("fcf");
constexpr int32_t mrt = sercom::stringToCode("mrt");
constexpr int32_t mbt = sercom::stringToCode("mbt");
constexpr int32_t tmt = sercom::stringToCode("tmt");
constexpr int32_t iro = sercom::stringToCode("iro");
constexpr int32_t oro = sercom::stringToCode("oro");
constexpr int32_t bvs = sercom::stringToCode("bvs");



void setup() {
  replaceTimer0WithTimer2();

  initIO();
  indication::color::black();
  indication::warningLed(255);

  INIT_UART(config::debug::baud);
  Serial.setTimeout(10);
  DEBUG(F("Remote\n"));

  initADC();

  // Calibrate analog inputs
  for (auto i = 0; i < 5; ++i) {
    readADC(pin::input::throttle);
    readADC(pin::input::pitch);
    readADC(pin::input::roll);
    getYawMode();
  }
  pitchStickCenter = readAnalogAverage(pin::input::pitch,
                                       config::adc::numOfReadings, config::adc::delayBetweenReadings);
  rollStickCenter = readAnalogAverage(pin::input::roll,
                                      config::adc::numOfReadings, config::adc::delayBetweenReadings);
#if DEBUG_CENTER_STICKS
  DEBUG("Sticks center's: pitch - "); DEBUG(pitchStickCenter);
  DEBUG("\troll - "); DEBUG(rollStickCenter); DEBUG("\n");
#endif

  directFlightMode = !getYawMode();

  // Communication initialization
  comm::rf24.initialize();
  comm::rf24.setPALevel(RF24_PA_MAX);
  comm::rf24.setDataRate(RF24_2MBPS);
  comm::rf24.setRetries(1, 15);
  comm::rf24.setCRCLength(2);
  // ~Communication initialization -----

  for (auto i = 0; i < 3; ++i) {
    indication::color::green(127);
    delay(40);
    indication::color::green(0);
    delay(40);
  }
  indication::warningLed(0);
  indication::color::black();
  DEBUG(F("Setup done.\n"));
}

void loop() {
  static uint32_t microsThisCycle = micros();
  microsThisCycle = micros();

#if DEBUG_LOOP_TIME
  static uint32_t loopTimeLUS = microsThisCycle;
  DEBUG("Loop time: "); DEBUG((microsThisCycle - loopTimeLUS)); DEBUG("\n");
  loopTimeLUS = microsThisCycle;
#endif


  // Read analog inputs
  inputState.throttle = readAnalogAverage(pin::input::throttle,
                                          config::adc::numOfReadings, config::adc::delayBetweenReadings);
  inputState.pitch = readAnalogAverage(pin::input::pitch,
                                       config::adc::numOfReadings, config::adc::delayBetweenReadings);
  inputState.roll = readAnalogAverage(pin::input::roll,
                                      config::adc::numOfReadings, config::adc::delayBetweenReadings);

  static int16_t throttle_last = inputState.throttle;
  inputState.throttle = inputState.throttle * config::throttle::lpfAlpha
                        + throttle_last * config::throttle::lpfOneMinusAlpha;
  throttle_last = inputState.throttle;

  static int16_t pitch_last = inputState.pitch;
  inputState.pitch = inputState.pitch * config::stick::lpfAlpha
                     + pitch_last * config::stick::lpfOneMinusAlpha;
  pitch_last = inputState.pitch;
  static int16_t roll_last = inputState.roll;
  inputState.roll = inputState.roll * config::stick::lpfAlpha
                    + roll_last * config::stick::lpfOneMinusAlpha;
  roll_last = inputState.roll;

#if DEBUG_RAW_ANALOG_INPUTS
  DEBUG("Throttle - "); DEBUG(inputState.throttle);
  DEBUG("\tpitch - "); DEBUG(inputState.pitch);
  DEBUG("\troll - "); DEBUG(inputState.roll); DEBUG("\n");
#endif
  // ~Read analog inputs -----


  // Read digital inputs
  if (!directFlightMode) {
    inputState.flightMode = (FlightMode)!getFlightMode();
  } else {
    inputState.flightMode = FlightMode::direct;
  }
  inputState.menu = getControlMode();
  inputState.yawControl = !getYawMode();
  inputState.fn = !getFn();
  inputState.sw = getSw();

#if DEBUG_DIGITAL_INPUTS
  if (inputState.flightMode == FlightMode::stabilize) { DEBUG("STAB\t"); }
  else if (inputState.flightMode == FlightMode::acro) { DEBUG("ACRO\t"); }
  else { DEBUG("DIRECT\t"); }
  if (inputState.menu) { DEBUG("MENU\t"); } else { DEBUG("FLIGHT\t"); }
  if (inputState.yawControl) { DEBUG("YAWING\t"); } else { DEBUG("ROLLING\t"); }
  if (inputState.fn) { DEBUG("FUNCTION"); }
  DEBUG("\n");
#endif
  // ~Read digital inputs -----


  if (droneSettings.telemetryType == (int8_t)PacketType::Telemetry
      || (int8_t)droneSettings.telemetryType < -1) {
    static float batteryVoltage_last = 0;
    static float altitude_last = 0;
    if (config::debug::batteryVoltage && !inputState.menu
        && (!compareFloat(droneStatus.batteryVoltage, batteryVoltage_last)
            || !compareFloat(droneStatus.altitude, altitude_last))) {
      DEBUG(F("Batt: ")); DEBUG(droneStatus.batteryVoltage); DEBUG(F("V"));
      batteryVoltage_last = droneStatus.batteryVoltage;
      DEBUG(F("\tAlt: ")); DEBUG(droneStatus.altitude); DEBUGLN(F("m"));
      altitude_last = droneStatus.altitude;
    }
  }


  if (inputState.menu) {
    bool increase = false;
    bool decrease = false;
    static CurrentSetting currentSetting = CurrentSetting::innerP;
    static uint32_t currentSettingLUS = 0;
    bool switched = false;
    if (microsThisCycle > currentSettingLUS + 200000) {
      currentSettingLUS = microsThisCycle;
      if (inputState.roll > 1000 && inputState.pitch > 100 && inputState.pitch < 923) {
        ++currentSetting; switched = true;
      } else if (inputState.roll < 23 && inputState.pitch > 100 && inputState.pitch < 923) {
        --currentSetting; switched = true;
      } else if (inputState.pitch > 1000 && inputState.roll > 100 && inputState.roll < 923) {
        increase = true;
        decrease = false;
      } else if (inputState.pitch < 23 && inputState.roll > 100 && inputState.roll < 923) {
        increase = false;
        decrease = true;
      }
    }

    Setting setting;
    setting._type = PacketType::Setting;
    setting.success = false;
    setting.request = false;
    setting.id = SettingId::dummy;

    switch (currentSetting) {
    case CurrentSetting::innerP: // 0.1-0.7 *0.25
      if (droneSettings.innerP < -0.1) {
        indication::color::red(1);
        setting.request = true;
        setting.id = SettingId::regInner_p;
        DEBUG(F("Pi"));
      } else {
        if (switched) { DEBUG(F("Pi: ")); DEBUGLN(droneSettings.innerP); switched = false; }
        indication::color::red( expMap[(uint8_t)round((droneSettings.innerP - 0.1) * 20)] ); // 20 = 12/0.6
        if (increase) { setting.value = droneSettings.innerP + 0.02; }
        else if (decrease) { setting.value = droneSettings.innerP - 0.02; }
        if (increase || decrease) setting.id = SettingId::regInner_p;
      }
      break;
    case CurrentSetting::innerYawP: // 0.1-1.0 *0.8
      if (droneSettings.innerYawP < -0.1) {
        indication::color::green(2);
        setting.request = true;
        setting.id = SettingId::regInner_yawP;
        DEBUG(F("Pi_yaw"));
      } else {
        if (switched) { DEBUG(F("Pi_yaw: ")); DEBUGLN(droneSettings.innerYawP); switched = false; }
        indication::color::green( expMap[(uint8_t)round((droneSettings.innerYawP - 0.1) * 13.33)] ); // 13.33 = 12/0.9
        if (increase) { setting.value = droneSettings.innerYawP + 0.02; }
        else if (decrease) { setting.value = droneSettings.innerYawP - 0.02; }
        if (increase || decrease) setting.id = SettingId::regInner_yawP;
      }
      break;
    case CurrentSetting::outerP: // 1.5-3.3 *2.5
      if (droneSettings.outerP < -0.1) {
        indication::color::blue(2);
        setting.request = true;
        setting.id = SettingId::regOuter_p;
        DEBUG(F("Po"));
      } else {
        if (switched) { DEBUG(F("Po: ")); DEBUGLN(droneSettings.outerP); switched = false; }
        indication::color::blue( expMap[(uint8_t)round((droneSettings.outerP - 1.5) * 6.66)] );
        if (increase) { setting.value = droneSettings.outerP + 0.02; }
        else if (decrease) { setting.value = droneSettings.outerP - 0.02; }
        if (increase || decrease) setting.id = SettingId::regOuter_p;
      }
      break;
    case CurrentSetting::outerI:
      if (droneSettings.outerI < -0.1) { // 0.0-1.8 *1.5
        indication::color::yellow(2);
        setting.request = true;
        setting.id = SettingId::regOuter_i;
        DEBUG(F("Io"));
      } else {
        if (switched) { DEBUG(F("Io: ")); DEBUGLN(droneSettings.outerI); switched = false; }
        indication::color::yellow( expMap[(uint8_t)round((droneSettings.outerI - 0.0) * 6.66)] );
        if (increase) { setting.value = droneSettings.outerI + 0.05; }
        else if (decrease) { setting.value = droneSettings.outerI - 0.05; }
        if (increase || decrease) setting.id = SettingId::regOuter_i;
      }
      break;
    case CurrentSetting::armsLevel:
      if (droneSettings.armsLevel < -1) {
        indication::color::cyan(2);
        setting.request = true;
        setting.id = SettingId::indication_armsLevel;
        DEBUG(F("Arms level"));
      } else {
        if (switched) { DEBUG(F("Arms level: ")); DEBUGLN(droneSettings.armsLevel); switched = false; }
        indication::color::cyan(expMap[droneSettings.armsLevel]);
        if (increase) { setting.value = droneSettings.armsLevel + 1; }
        else if (decrease) { setting.value = droneSettings.armsLevel - 1; }
        if (increase || decrease) setting.id = SettingId::indication_armsLevel;
      }
      break;
    case CurrentSetting::commonAlpha:
      if (droneSettings.commonAlpha < -1) {
        indication::color::magenta(2);
        setting.request = true;
        setting.id = SettingId::imuLpf_common;
        DEBUG(F("LPF alpha"));
      } else {
        if (switched) { DEBUG(F("LPF alpha: ")); DEBUGLN(droneSettings.commonAlpha); switched = false; }
        indication::color::magenta(expMap[droneSettings.commonAlpha * 2]);
        if (increase) { setting.value = droneSettings.commonAlpha + 1; }
        else if (decrease) { setting.value = droneSettings.commonAlpha - 1; }
        if (increase || decrease) setting.id = SettingId::imuLpf_common;
      }
      break;
    case CurrentSetting::telemetryType:
      if (droneSettings.telemetryType < -1) {
        indication::color::white(2);
        setting.request = true;
        setting.id = SettingId::commTelemetry_type;
        DEBUG(F("Telemetry type"));
      } else {
        if (switched) { DEBUG(F("Telemetry type: ")); DEBUGLN(droneSettings.telemetryType); switched = false; }
        indication::color::white(expMap[(droneSettings.telemetryType - 1) * 4]);
        if (increase) { setting.value = droneSettings.telemetryType + 1; }
        else if (decrease) { setting.value = droneSettings.telemetryType - 1; }
        if (increase || decrease) setting.id = SettingId::commTelemetry_type;
      }
      break;
    default:
      currentSetting = CurrentSetting::innerP;
      break;
    }
    if (setting.id != SettingId::dummy) {
      comm::rf24.stopListening();
      if (comm::rf24.write(&setting, sizeof(setting))) {
        if (!increase && !decrease) DEBUG(F(" ->\n"));
      } else {
        if (!increase && !decrease) DEBUG(F(" x\n"));
      }
      comm::rf24.startListening();
    }
  }

  // static uint32_t lampCheckLUS = microsThisCycle;
  // if (microsThisCycle > lampCheckLUS + 100000) {
  //   lampCheckLUS = microsThisCycle;
  //   if (droneSettings.lampState < -0.1) {
  //     Setting setting;
  //     setting._type = PacketType::Setting;
  //     setting.success = false;
  //     setting.request = true;
  //     setting.id = SettingId::indication_lamp;
  //     setting.value = 0.0;
  //     if (comm::rf24.write(&setting, sizeof(setting))) {
  //       DEBUG(F("Requested the state of the lamp.\n"));
  //     }
  //   } else if (inputState.sw != (bool)droneSettings.lampState) {
  //     Setting setting;
  //     setting._type = PacketType::Setting;
  //     setting.success = false;
  //     setting.request = false;
  //     setting.id = SettingId::indication_lamp;
  //     setting.value = (float)inputState.sw;
  //     if (comm::rf24.write(&setting, sizeof(setting))) { indication::color::white(HALF_ON); }
  //   }
  // }


  static uint32_t commandLUS = microsThisCycle;
  if (microsThisCycle - commandLUS > config::communication::commandPeriod) {
    commandLUS = microsThisCycle;
    Command command;
    command._type = PacketType::Command;
    command._type = (PacketType)0;
    static uint32_t messageId = 0;
    command.throttle = clamp(round(inputState.throttle >> 3), 0, 127);
    command.flightMode = inputState.flightMode;
    float multiplier = 1.0f;
    float exponent = 1.0f;
    // ((x*0.02)^(2))
    // ((x*0.012)^(2))
    if (inputState.fn == true) {
      if (command.flightMode == FlightMode::stabilize) multiplier = 1.25; // 50 deg
      else if (command.flightMode == FlightMode::acro) multiplier = 1.5; // 165 deg/s
      else multiplier = 1.1;
    }
    if (directFlightMode) {
      multiplier *= 0.1;
      command.flightMode = FlightMode::direct;
    } else if (inputState.flightMode == FlightMode::stabilize) {
      multiplier *= 0.078; // 40 deg
      // multiplier *= 0.011;
      // exponent = 2.0f;
    } else if (inputState.flightMode == FlightMode::acro) {
      multiplier *= 0.21; // 110 deg/s
    } else {
      multiplier *= 0.1;
    }
    command.pitch = clamp(inputState.pitch - pitchStickCenter, -pitchStickCenter, pitchStickCenter);
    command.pitch = -1 * round(pow(command.pitch * multiplier, exponent));
    command.roll = -clamp(inputState.roll - rollStickCenter, -rollStickCenter, rollStickCenter);
    command.roll = -1 * round(pow(command.roll * multiplier, exponent));
    if (inputState.yawControl) {
      command.yaw = command.roll * -1;
      if (command.flightMode == FlightMode::stabilize) command.yaw *= 2.3f;
      else if (command.flightMode == FlightMode::acro) command.yaw *= 1.1f;
      command.roll = 0;
    } else {
      command.yaw = 0;
    }
    command.altitudeHold = inputState.sw;
    if (inputState.menu) {
      command.flightMode = FlightMode::stabilize;
      command.pitch = 0;
      command.roll = 0;
      command.yaw = 0;
    }
    command.messageId = messageId++;
    command.senderId = 33;

#if DEBUG_COMMAND
    // DEBUG("Pitch: state("); DEBUG(inputState.pitch);
    // DEBUG(")\tcenter("); DEBUG(inputState.pitch - pitchStickCenter);
    // DEBUG(")\tclamp("); DEBUG(clamp(inputState.pitch - pitchStickCenter, -pitchStickCenter, pitchStickCenter));
    // DEBUG(")\tclamp/10("); DEBUG(clamp(inputState.pitch - pitchStickCenter, -pitchStickCenter, pitchStickCenter) / 10);
    // DEBUG(")\n");

    if (command.flightMode == FlightMode::stabilize) { DEBUG("STAB - "); }
    else if (command.flightMode == FlightMode::acro) { DEBUG("ACRO - "); }
    else { DEBUG("DIRECT - "); }
    DEBUG("TH: "); DEBUG(command.throttle); DEBUG(", P: "); DEBUG(command.pitch);
    DEBUG(", R: "); DEBUG(command.roll); DEBUG(", Y: "); DEBUGLN(command.yaw);
#endif
    comm::rf24.write(&command, sizeof(command));
  }

  static uint32_t ackLUS = microsThisCycle;
  if (comm::rf24.available()) {
    while (comm::rf24.available()) {
      ackLUS = microsThisCycle;
      status::communication = Status::normal;
      uint8_t rawMessage[32];
      comm::rf24.read(&rawMessage, 32);
      switch (rawMessage[0]) {
      case 0: break;
      case (uint8_t)PacketType::Telemetry:
        // DEBUGLN(F("T"));
        Telemetry telemetry;
        memcpy(&telemetry, &rawMessage, sizeof(telemetry));
        droneStatus.batteryVoltage = telemetry.batteryVoltage;
        droneStatus.altitude = telemetry.altitude;
        indication::greenLed(0);
        indication::blueLed(0);
        break;
      case (uint8_t)PacketType::Telemetry_regulation:
        // DEBUGLN(F("T reg"));
        Telemetry_regulation telemetry_regulation;
        memcpy(&telemetry_regulation, &rawMessage, sizeof(telemetry_regulation));
        droneStatus.batteryVoltage = telemetry_regulation.batteryVoltage;
        if (config::debug::telemetryRegulation && inputState.menu == false
            && (status::battery == Status::normal || status::battery == Status::debugging)) {
          DEBUG(telemetry_regulation.commandRoll); DEBUG(F(", "));
          if (inputState.flightMode == FlightMode::acro) {DEBUG(telemetry_regulation.avRoll);}
          else if (inputState.flightMode == FlightMode::stabilize) {DEBUG(telemetry_regulation.attitudeRoll);}
          DEBUGLN();
        }
        indication::greenLed(0);
        indication::blueLed(0);
        break;
      case (uint8_t)PacketType::Telemetry_imu:
        // DEBUGLN(F("T imu"));
        Telemetry_imu telemetry_imu;
        memcpy(&telemetry_imu, &rawMessage, sizeof(telemetry_imu));
        droneStatus.batteryVoltage = telemetry_imu.batteryVoltage;
        droneStatus.angularVelocity.x = telemetry_imu.angularVelocity.x;
        droneStatus.angularVelocity.y = telemetry_imu.angularVelocity.y;
        droneStatus.angularVelocity.z = telemetry_imu.angularVelocity.z;
        droneStatus.acceleration.x = telemetry_imu.acceleration.x;
        droneStatus.acceleration.y = telemetry_imu.acceleration.y;
        droneStatus.acceleration.z = telemetry_imu.acceleration.z;
        droneStatus.attitude.pitch = telemetry_imu.attitude.pitch;
        droneStatus.attitude.roll = telemetry_imu.attitude.roll;
        if (inputState.menu == false
            && (status::battery == Status::normal || status::battery == Status::debugging)) {
          if (inputState.flightMode == FlightMode::stabilize) {
            if (config::debug::telemetryImu) {
              DEBUG(telemetry_imu.attitude.pitch); DEBUG(F(", "));
              DEBUG(telemetry_imu.attitude.roll); DEBUGLN();
            }
            // if (status::battery == Status::normal) {
            indication::greenLed(abs(telemetry_imu.attitude.pitch));
            indication::blueLed(abs(telemetry_imu.attitude.roll));
            // }
          } else {
            if (config::debug::telemetryImu) {
              DEBUG(telemetry_imu.angularVelocity.x); DEBUG(F(", "));
              DEBUG(telemetry_imu.angularVelocity.y); DEBUG(F(", "));
              DEBUG(telemetry_imu.angularVelocity.z); DEBUGLN();
            }
            // if (status::battery == Status::normal) {
            indication::greenLed(abs(telemetry_imu.angularVelocity.x));
            indication::blueLed(abs(telemetry_imu.angularVelocity.y));
            // }
          }
        } //if (inputState.menu == false && status::battery == Status::normal)
        break;
      case (uint8_t)PacketType::Telemetry_motors:
        // DEBUGLN(F("T motors"));
        Telemetry_motors telemetry_motors;
        memcpy(&telemetry_motors, &rawMessage, sizeof(telemetry_motors));
        droneStatus.tl = telemetry_motors.tl;
        droneStatus.tr = telemetry_motors.tr;
        droneStatus.bl = telemetry_motors.bl;
        droneStatus.br = telemetry_motors.br;
        if (config::debug::telemetryMotors && inputState.menu == false) {
          {
            uint8_t newLines = 0;
            while (newLines++ < 12) { DEBUG(F("\n")); }
          }
          DEBUG(telemetry_motors.tl); DEBUG(F("\t")); DEBUG(telemetry_motors.tr); DEBUG(F("\n"));
          DEBUG(telemetry_motors.bl); DEBUG(F("\t")); DEBUG(telemetry_motors.br); DEBUG(F("\n"));
        }
        indication::greenLed(0);
        indication::blueLed(0);
        break;
      case (uint8_t)PacketType::Setting:
        Setting setting;
        memcpy(&setting, &rawMessage, sizeof(setting));
        DEBUG(F("<- "));
        switch (setting.id) {
        case SettingId::dummy:
          DEBUG(F("dummy"));
          break;
        case SettingId::imuLpfAcc_alpha:
          DEBUG(F("LPFacc alpha"));
          droneSettings.accAlpha = setting.value;
          break;
        case SettingId::imuLpfAv_alpha:
          DEBUG(F("LPFav alpha"));
          droneSettings.avAlpha = setting.value;
          break;
        case SettingId::commTelemetry_type:
          DEBUG(F("Telemetry type"));
          droneSettings.telemetryType = setting.value;
          break;
        case SettingId::regInner_p:
          droneSettings.innerP = setting.value;
          DEBUG(F("Pi"));
          break;
        case SettingId::regInner_yawP:
          droneSettings.innerYawP = setting.value;
          DEBUG(F("Pi_yaw"));
          break;
        case SettingId::regOuter_p:
          droneSettings.outerP = setting.value;
          DEBUG(F("Po"));
          break;
        case SettingId::regOuter_i:
          droneSettings.outerI = setting.value;
          DEBUG(F("Io"));
          break;
        case SettingId::imuLpf_common:
          droneSettings.commonAlpha = setting.value;
          DEBUG(F("LPF"));
          break;
        case SettingId::indication_lamp:
          droneSettings.lampState = setting.value;
          DEBUG(F("Lamp state"));
          break;
        case SettingId::indication_armsLevel:
          droneSettings.armsLevel = setting.value;
          DEBUG(F("Arms level"));
          break;
        case SettingId::regInner_outputLimit:
          DEBUG(F("Inner output limit"));
          break;
        case SettingId::regOuter_outputLimit:
          DEBUG(F("Outer output limit"));
          break;
        case SettingId::reg_minRegThrottle:
          DEBUG(F("Min regulation throttle"));
          break;
        case SettingId::reg_maxBaseThrottle:
          DEBUG(F("Max base throttle"));
          break;
        default:
          DEBUG(F("unrecognized setting code"));
          break;
        }
        DEBUG(F(": ")); DEBUG(setting.value, 3);
        if (!setting.success) { DEBUG(F("\tchange failed (probably out of bounds).\n")); }
        else {
          indication::color::white(255);
          delay(1);
        }
        DEBUGLN();
        break;
      default:
        DEBUG(F("Received unrecognized message type "));
        DEBUG(rawMessage[0]); DEBUGLN(F(". Igonring"));
        comm::rf24.reset();
        break;
      }
    }
  }


  if (microsThisCycle > ackLUS + 105000) {
    status::communication = Status::error;
    DEBUG(F("."));
  }


// Indication
  static uint32_t indicationLUS = microsThisCycle;
  if (microsThisCycle > indicationLUS + config::indication::period && !inputState.menu) {
    indicationLUS = microsThisCycle;
    if (config::battery::indicateLowVoltage) {
      if (droneStatus.batteryVoltage > 10.1) { status::battery = Status::normal; }
      else if (droneStatus.batteryVoltage > 5.0) { status::battery = Status::warning; }
      else { status::battery = Status::debugging; }
      if (status::battery == Status::normal) {
        float batteryRelative = 12.6 - droneStatus.batteryVoltage;
        indication::redLed(expMap[clamp((uint8_t)round(batteryRelative * 4.8), 0, 12)]);
        // indication::greenLed(OFF);
        // indication::blueLed(OFF);
      } else if (status::battery == Status::warning || directFlightMode) {
        indication::redLedToggle();
        // indication::greenLed(OFF);
        // indication::blueLed(OFF);
      } else if (status::battery == Status::debugging) {
        indication::redLed(OFF);
        // indication::greenLed(OFF);
        // indication::blueLed(OFF);
      } else {
        status::battery = Status::error;
      }
    }
    if (status::communication != Status::normal) {
      indication::warningLedToggle();
      indication::greenLed(OFF);
      indication::blueLed(OFF);
    } else {
      indication::warningLed(OFF);
    }
  }
// ~Indication -----


// Serial commands
  // s.iry-0.1;
  // s.iry-0.01;
  // s.iry-0.001;
  // s.iry-0.0001;
  // 0123456789TE
  // s.iry-1.2;
  // s.iry-12.3;
  // s.iry-123.4;
  // s.iry-1234.5;
  // 0123456789TE
  // s.iry-0;
  // s.iry-12;
  // s.iry-123;
  // s.iry-1234;
  // s.iry-12345;
  // s.iry-123456;
  // 0123456789TE
  if (Serial.available()) {
    int8_t commandLength = 13;
    char command[commandLength];
    Serial.readBytesUntil('\n', command, commandLength);
    while (Serial.available()) Serial.read();

    if (sercom::isValid(command)) {
      Setting setting;
      setting._type = PacketType::Setting;
      setting._type = (PacketType)3;
      setting.success = false;
      if (command[0] == 's') {
        setting.request = false;
        DEBUG(F("Changing "));
      } else {
        setting.request = true;
        DEBUG(F("Requesting "));
      }
      setting.id = SettingId::dummy;
      uint32_t commandCode = sercom::commandToCode(command);
      switch (commandCode) {
      case irp: {
        setting.id = SettingId::regInner_p;
        setting.value = sercom::commandToFloat(command);
        DEBUG(F("Inner Regulator Proportional"));
        break;
      }
      case iry: {
        setting.id = SettingId::regInner_yawP;
        setting.value = sercom::commandToFloat(command);
        DEBUG(F("Inner Regulator Yaw"));
        break;
      }
      case orp: {
        setting.id = SettingId::regOuter_p;
        setting.value = sercom::commandToFloat(command);
        DEBUG(F("Outer Regulator Proportional"));
        break;
      }
      case ori: {
        setting.id = SettingId::regOuter_i;
        setting.value = sercom::commandToFloat(command);
        DEBUG(F("Outer Regulator Integral"));
        break;
      }
      case clf: {
        setting.id = SettingId::imuLpf_common;
        setting.value = sercom::commandToFloat(command);
        DEBUG(F("Common Low-pass Filter"));
        break;
      }
      case glf: {
        setting.id = SettingId::imuLpfAv_alpha;
        setting.value = sercom::commandToFloat(command);
        DEBUG(F("Gyroscope Low-pass Filter"));
        break;
      }
      case alf: {
        setting.id = SettingId::imuLpfAcc_alpha;
        setting.value = sercom::commandToFloat(command);
        DEBUG(F("Accelerometer Low-pass Filter"));
        break;
      }
      case fcf: {
        setting.id = SettingId::imuComplementary_alpha;
        setting.value = sercom::commandToFloat(command);
        DEBUG(F("Fusion Complementary Filter"));
        break;
      }
      case mrt: {
        setting.id = SettingId::reg_minRegThrottle;
        setting.value = sercom::commandToInt(command);
        DEBUG(F("Maximum Regulation Throttle"));
        break;
      }
      case mbt: {
        setting.id = SettingId::reg_maxBaseThrottle;
        setting.value = sercom::commandToInt(command);
        DEBUG(F("Minimum Base Throttle"));
        break;
      }
      case tmt: {
        setting.id = SettingId::commTelemetry_type;
        setting.value = sercom::commandToInt(command);
        DEBUG(F("TeleMetry Type"));
        break;
      }
      case iro: {
        setting.id = SettingId::regInner_outputLimit;
        setting.value = sercom::commandToInt(command);
        DEBUG(F("Inner Regulator Output limit"));
        break;
      }
      case oro: {
        setting.id = SettingId::regOuter_outputLimit;
        setting.value = sercom::commandToInt(command);
        DEBUG(F("Outer Regulator Output limit"));
        break;
      }
      case bvs: {
        setting.id = SettingId::dummy;
        DEBUG(F("Battery Voltage Status ")); DEBUG(droneStatus.batteryVoltage); DEBUGLN(F("."));
        break;
      }
      default:
        setting.id = SettingId::dummy;
        DEBUG(F("!unknown command code ")); DEBUG(commandCode); DEBUG(F("! "));
        break;
      }
      if (setting.id != SettingId::dummy) {
        if (setting.request == false) {
          DEBUG(F(" to ")); DEBUG(setting.value); DEBUGLN(F("."));
        } else { DEBUGLN(F(".")); }
        comm::rf24.stopListening();
        setting._type = PacketType::Setting;
        comm::rf24.write(&setting, sizeof(setting));
        comm::rf24.startListening();
      }
    } else if (command[0] == 'h') {
      DEBUGLN(F("\nLIST OF AVAILABLE COMMANDS:"));
      DEBUGLN(F(" Drone settings:"));
      DEBUGLN(F("  irp - Inner Regulator Proportional"));
      DEBUGLN(F("  iry - Inner Regulator Yaw"));
      DEBUGLN(F("  orp - Outer Regulator Proportional"));
      DEBUGLN(F("  ori - Outer Regulator Integral"));
      DEBUGLN(F("  clf - Common Low-pass Filter"));
      DEBUGLN(F("  glf - Gyroscope Low-pass Filter"));
      DEBUGLN(F("  alf - Accelerometer Low-pass Filter"));
      DEBUGLN(F("  fcf - Fusion Complementary Filter"));
      DEBUGLN(F("  mrt - Maximum Regulation Throttle"));
      DEBUGLN(F("  mbt - Minimum Base Throttle"));
      DEBUGLN(F("  tmt - TeleMetry Type"));
      DEBUGLN(F("  iro - Inner Regulator Output limit"));
      DEBUGLN(F("  oro - Outer Regulator Output limit"));
      DEBUGLN(F("  bvs - Battery Voltage Status"));
      DEBUGLN(F(" Remote settings:"));
      DEBUGLN();
    } else {
      DEBUGLN(F("Invalid command! Correct format: \"(s/g).(3 letter identifier)-(1..6 digit value);\"."));
    }
  } // if(Serial.available())

// ~Serial commands -----

}
