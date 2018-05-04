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
float recordRelativeAltitude = 0.0f;
const float seaLevelPressure = 101325.0f;

using namespace m328;

float verticalVelocity = 0.0f;
int16_t verticalVelocity_setpoint = 0;
int16_t verticalVelocity_output = 0;
PID verticalVelocity_pid(&verticalVelocity_setpoint, &verticalVelocity, &verticalVelocity_output);

bool altitudeHold = false;
uint32_t altitudeHoldSetTime = 0;
const uint32_t altitudeHoldTimeOut = 5000000;

float recordClimbSpeed = 0.0f;
float recordFallSpeed = 0.0f;

const uint16_t recordClimbSpeed_address = 250;
const uint16_t recordFallSpeed_address = 260;
const uint16_t recordRelativeAltitude_address = 280;

struct Str {
  uint16_t b_mv;
  uint16_t rc;
};

// #include "AK8963_I2C.hpp"
// AK8963_I2C compass;
// float mx = 0.0;
// float my = 0.0;
// float mz = 0.0;


namespace imu {
MPU9255 mpu6050(config::imu::i2cAddress);
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

namespace pid {

namespace inner {
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
} //namespace inner

namespace outer {
PID pitch(&command.pitch, &attitude.pitch, &inner::setpoint::pitch);
PID roll(&command.roll, &attitude.roll, &inner::setpoint::roll);
} //namespace outer

} //namespace pid


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
// #define DT round(config::cycleTime / 1000000);


void setup() {
  bool success = true;

  replaceTimer0WithTimer2();

  initIO();
  indication::signal(OFF);
  indication::warning(ON);
  indication::arms(OFF);

  Wire.begin();
  Wire.setClock(400000);

  INIT_UART(config::debug::baud);
  uart::initialize(config::debug::baud);
  // Serial.begin(config::debug::baud);
  DEBUGLN("FC2.\n");

  config::init();
#if DEBUG_SETTINGS
  DEBUGLN("  SETTINGS");
  DEBUG("  Telemetry type: "); DEBUGLN(config::communication::telemetry::type());
  DEBUG("  Pi: "); DEBUGLN(config::regulation::inner::P());
  DEBUG("  Pi_yaw: "); DEBUGLN(config::regulation::inner::yawP());
  DEBUG("  Po: "); DEBUGLN(config::regulation::outer::P());
  DEBUG("  Io: "); DEBUGLN(config::regulation::outer::I());
  DEBUG("  Arms level (0-12): "); DEBUGLN(config::indication::armsLevel());
  DEBUG("  Lamp: "); if (config::indication::lamp()) {DEBUGLN("ON");} else {DEBUGLN("OFF");}
  DEBUGLN("  -----\n");
#endif

  initADC();

  // IMU initialization
  DEBUG("IMU");
  success &= imu::mpu6050.initialize();
  // imu::mpu6050.setDLPFMode(config::imu::lowPassFilter::common());
  DEBUG(" done. T = "); DEBUG(imu::mpu6050.getTemperature()); DEBUGLN("degC.");
  // Wire.setClock(400000);
  Wire.setClock(800000);
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
  success &= pid::inner::pitch.setTunings(config::regulation::inner::P());
  success &= pid::inner::roll.setTunings(config::regulation::inner::P());
  success &= pid::inner::yaw.setTunings(config::regulation::inner::yawP(),
                                        config::regulation::inner::yawI);
  success &= pid::inner::pitch.setOutputLimits(config::regulation::inner::outputLimit);
  success &= pid::inner::roll.setOutputLimits(config::regulation::inner::outputLimit);
  success &= pid::inner::yaw.setOutputLimits(config::regulation::inner::outputLimit);
  success &= pid::inner::yaw.setIntegralLimit(2);
  pid::inner::pitch.on(); pid::inner::roll.on(); pid::inner::yaw.on();

  success &= pid::outer::pitch.setTunings(config::regulation::outer::P(), config::regulation::outer::I());
  success &= pid::outer::roll.setTunings(config::regulation::outer::P(), config::regulation::outer::I());
  pid::outer::pitch.setUpdateRate(config::regulation::outer::updateRate);
  pid::outer::roll.setUpdateRate(config::regulation::outer::updateRate);
  success &= pid::outer::pitch.setOutputLimits(config::regulation::outer::outputLimit);
  success &= pid::outer::roll.setOutputLimits(config::regulation::outer::outputLimit);
  success &= pid::outer::pitch.setIntegralLimit(64);
  success &= pid::outer::roll.setIntegralLimit(64);
  pid::outer::pitch.on(); pid::outer::roll.on();

  // eeprom::write((const uint16_t)155, (const float)0.5f);
  // eeprom::write((const uint16_t)160, (const float)0.2f);
  verticalVelocity_pid.setTunings(0.0f, 15.0f, 1.0f);
  verticalVelocity_pid.setUpdateRate(5000);
  verticalVelocity_pid.setOutputLimits(85); // more accurate velocity, outer pid...
  verticalVelocity_pid.setIntegralLimit(0, 85);
  verticalVelocity_pid.on();
  // DEBUGLN(" done.");
  // ~Regulation initialization -----


#  define ZERO_RECORDS false // zero records only if V < 5
#  if ZERO_RECORDS
  eeprom_busy_wait();
  eeprom::write(recordClimbSpeed_address, 0.0f);
  eeprom_busy_wait();
  eeprom::write(recordFallSpeed_address, 0.0f);
  eeprom_busy_wait();
  eeprom::write(recordRelativeAltitude_address, 0.0f);
#  endif
  for (uint16_t i = 300; i < 1010; i += 4) {
    uint16_t b = 0;
    uint16_t c = 0;
#   if ZERO_RECORDS
    eeprom_busy_wait();
    eeprom::update(i, b);
    eeprom_busy_wait();
    eeprom::update(i + 2, c);
#   endif
    eeprom_busy_wait();
    eeprom::read(i, b);
    eeprom_busy_wait();
    eeprom::read(i + 2, c);
    Serial.print(b); Serial.print("\t");
    Serial.println(c);
  }

  bmp.initialize();
  bmp.setPressureOversampleRatio(16);
  bmp.setTemperatureOversampleRatio(2);
  bmp.setFilterRatio(16);
  bmp.setStandby(0);
  bmp.setEnabled(true);

  float p = 0.0f;
  float t = 0.0f;
  for (uint16_t i = 0; i < 500; ++i) {
    bmp.read(p, t);
  }
  float pSum = 0.0f;
  float tSum = 0.0f;
  const uint16_t samples = 400;
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
  uart::print("Pa, A: "); uart::print(takeOffAltitude);
  eeprom::read(recordRelativeAltitude_address, recordRelativeAltitude);
  uart::print("m, RRA: "); uart::print(recordRelativeAltitude); uart::print("m\n");

  eeprom::read(recordClimbSpeed_address, recordClimbSpeed);
  uart::print("Rec climb speed: "); uart::print(recordClimbSpeed); uart::print("kph\n");

  eeprom::read(recordFallSpeed_address, recordFallSpeed);
  uart::print("Rec fall speed: "); uart::print(recordFallSpeed); uart::print("kph\n");

  // compass.powerUp();
  // compass.initialize();
  // compass.setResolution(BITS_16);
  // compass.setMode(Mode::CONTINUOUS_MEASUREMENT_100HZ);
  // // self test if fail disable mag
  // Wire.setClock(800000);


  if (!success) { DEBUG("Wrong initialization configuration! Terminated."); exit(0); }
  indication::signal(ON);
  indication::warning(OFF);
  indication::lamp(config::indication::lamp());
  // indication::arms(0);
  // DEBUGLN("Setup done.\n----------");
} //void setup()


void loop() {
  static uint32_t microsThisCycle = micros();
  // static uint32_t microsNow = microsThisCycle;

#if DEBUG_LOOP_TIME
  uart::print(micros() - microsThisCycle);
  // indication::toggleLamp();
#endif

  microsThisCycle = micros();


  // Regulation
  static uint32_t innerComputeLUS = microsThisCycle;
  WAIT_FOR_CYCLE(innerComputeLUS);
  innerComputeLUS = micros();
  pid::inner::yaw.compute();
  pid::inner::pitch.compute();
  pid::inner::roll.compute();

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
    pid::outer::pitch.compute();
    pid::outer::roll.compute();
  }

#if DEBUG_PID_PITCH
  if (!innerCycles % 3) {
    if (command.flightMode == FlightMode::stabilize) {
      uart::print(command.pitch);
      uart::print(","); uart::print(attitude.pitch);
    } else {
      uart::print(","); uart::print(pid::inner::setpoint::pitch);
      uart::print(","); uart::print(angularVelocity.y);
    }
    uart::print(","); uart::print(pid::inner::output::pitch); uart::print("\n");
  }
#endif
#if DEBUG_PID_ROLL
  if (!innerCycles % 3) {
    if (command.flightMode == FlightMode::stabilize) {
      uart::print(command.roll);
      uart::print(","); uart::print(attitude.roll);
    }
    uart::print(","); uart::print(pid::inner::setpoint::roll);
    uart::print(","); uart::print(angularVelocity.x);
    uart::print(","); uart::print(pid::inner::output::roll); uart::print("\n");
  }
#endif
#if DEBUG_PID_YAW
  if (!innerCycles % 3) {
    uart::print(command.yaw);
    uart::print(", "); uart::print(avYaw);
    uart::print(", "); uart::print(pid::inner::output::yaw);
    uart::print("\n");
  }
#endif
  // ~Regulation -----


  // Attitude -----
  // Fetch
  imu::mpu6050.getMotion(&angularVelocity, &acceleration);



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


// #define COMPLEMENTARY_FUSION false
// #if COMPLEMENTARY_FUSION
//   // Accelerometer angle
//   float x_sqr = square((float)acceleration.x);
//   float y_sqr = square((float)acceleration.y);
//   float z_sqr = square((float)acceleration.z);

//   float roll_a = atan2( (float) - acceleration.x,
//                         sqrt(y_sqr + z_sqr))
//                  * toDegrees;
//   float pitch_a = atan2( (float) acceleration.y,
//                          sign(acceleration.z) * sqrt(z_sqr + (config::imu::epsilon * x_sqr)))
//                   * toDegrees;

//   // Fusion
//   static uint32_t fusionLUS = microsThisCycle;
//   microsNow = micros();
//   float dt = (float)(microsNow - fusionLUS) / 1000000.0;

//   attitude.pitch = (attitude.pitch + angularVelocity.x * dt) * config::imu::complementary::alpha()
//                    + pitch_a * config::imu::complementary::oneMinusAlpha;
//   attitude.roll = (attitude.roll + angularVelocity.y * dt) * config::imu::complementary::alpha()
//                   + roll_a * config::imu::complementary::oneMinusAlpha;
//   fusionLUS = microsNow;
//   // { LPF attitude }
// #endif
// #if DEBUG_ACCELEROMETER_ANGLE
//   if (innerCycles == 0) {
//     DEBUG(pitch_a); DEBUG(", "); DEBUG(angularVelocity.x);
//     DEBUG(", ");  DEBUG(roll_a); DEBUG(", "); DEBUGLN(angularVelocity.y);
//   }
// #endif

#define MAHONY_FUSION true
#if MAHONY_FUSION
  static uint32_t mahonyLUS = micros();
  WAIT_FOR_CYCLE(mahonyLUS);
  // Serial.println(micros() - mahonyLUS);
  mahonyLUS = micros();
  float mahonyYaw, mahonyPitch, mahonyRoll;
  mahony.update(mahonyYaw, mahonyPitch, mahonyRoll,
                acceleration.x, acceleration.y, acceleration.z,
                angularVelocity.x * DEG_TO_RAD, angularVelocity.y * DEG_TO_RAD, angularVelocity.z * DEG_TO_RAD);
  // mahony.update(mahonyYaw, mahonyPitch, mahonyRoll,
  //               acceleration.x, acceleration.y, acceleration.z,
  //               angularVelocity.x * DEG_TO_RAD, angularVelocity.y * DEG_TO_RAD, angularVelocity.z * DEG_TO_RAD,
  //               my, mx, mz);
  attitude.roll = mahonyPitch * RAD_TO_DEG;
  attitude.pitch = mahonyRoll * RAD_TO_DEG;
  float mYaw = mahonyYaw * RAD_TO_DEG;

  // float Xh = mx * cos(attitude.pitch) + my * sin(attitude.roll) * sin(attitude.pitch) - mz * cos(attitude.roll) * sin(attitude.pitch);
  // float Yh = my * cos(attitude.roll) + mz * sin(attitude.roll);
  // float yawmag = atan2(Yh, Xh) + M_PI;
  // Serial.println(yawmag);

  static uint32_t dVelocityLUS = microsThisCycle;
  static float attitudeYawLast = mYaw;
  uint32_t us = micros();
  float vDt = (us - dVelocityLUS + 4) / 1000000.0f;
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
  if (throttle > config::regulation::minimumRegulationThrottle) {
    if (throttle > config::regulation::maximumBaseThrottle) throttle = config::regulation::maximumBaseThrottle;
    tl_p = clamp(throttle
                 + pid::inner::output::pitch + pid::inner::output::roll - pid::inner::output::yaw,
                 0, 127);
    tr_p = clamp(throttle
                 + pid::inner::output::pitch - pid::inner::output::roll + pid::inner::output::yaw,
                 0, 127);
    bl_p = clamp(throttle
                 - pid::inner::output::pitch + pid::inner::output::roll + pid::inner::output::yaw,
                 0, 127);
    br_p = clamp(throttle
                 - pid::inner::output::pitch - pid::inner::output::roll - pid::inner::output::yaw,
                 0, 127);
  } else {
    tl_p = throttle;
    tr_p = throttle;
    bl_p = throttle;
    br_p = throttle;
    pid::outer::pitch.unwind();
    pid::outer::roll.unwind();
    pid::inner::yaw.unwind();
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

  // if (compass.read(&mx, &my, &mz)) mz = 0.0f - mz;
  // compass.read(&mx, &my, &mz);
  // if (compass.overflow()) {
  //   mx = 0.0f; my = 0.0f; mz = 0.0f;
  // }
  // uart::print(mx); uart::print(",");
  // uart::print(my); uart::print(",");
  // uart::println(-mz);
  // Wire.setClock(800000);

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
      // case PacketType::Telemetry_imu:
      //   comm::rf24.reset();
      //   DEBUGLN("?t_imu");
      //   break;
      // case PacketType::Telemetry_motors:
      //   comm::rf24.reset();
      //   DEBUGLN("?t_motors");
      //   break;
      default:
        comm::rf24.reset();
        DEBUG("unrec "); DEBUGLN(rawMessage[0]);
        break;
      }
    }
  } else if (microsThisCycle - lastCommandLUS > config::communication::commandTimeout) {
    throttle = 0;
    pid::outer::pitch.on();
    pid::outer::roll.on();
    command.pitch = 0;
    command.roll = 0;
    command.yaw = 0;
    altitudeHold = false;
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


  static float p = takeOffPressure;
  static float t = 0.0;
  if (innerCycles == 2) { //700us
    float newP = takeOffPressure;
    bmp.read(newP, t);
    if (newP < p - 5000.0 || newP > p + 5000.0) newP = p;
    p = newP * 0.07 + p * 0.93;
    // Serial.print("P: "); Serial.print(p); Serial.print("hPa\t");
  } else if (innerCycles == 3) { //400us
    float absoluteAltitude = calculateAltitude(p, seaLevelPressure, t);
    relativeAltitude = absoluteAltitude - takeOffAltitude;

    static float relativeAltitudeLastFilter = relativeAltitude;
    relativeAltitude = relativeAltitude * 0.08 + relativeAltitudeLastFilter * 0.92;
    relativeAltitudeLastFilter = relativeAltitude;
    // Serial.print("A: "); Serial.print(absoluteAltitude); Serial.print("m\t");
    // Serial.print("RA: "); Serial.print(relativeAltitude); Serial.print("m\t");

    static bool recorded = true;
    if (relativeAltitude > recordRelativeAltitude || recorded == false) {
      recordRelativeAltitude = relativeAltitude;
      recorded = eeprom::write(recordRelativeAltitude_address, recordRelativeAltitude);
    } else if (absoluteAltitude < takeOffAltitude - 1.5f) {
      takeOffAltitude = absoluteAltitude;
    }

    // // kinda good
    // static float relativeAltitudeLast = relativeAltitude;
    // static uint32_t verticalVelocityLUS = microsThisCycle;
    // float vvDt = (float)(micros() - verticalVelocityLUS) / 1000000.0f;
    // verticalVelocityLUS = micros();
    // verticalVelocity = ((float)(relativeAltitude - relativeAltitudeLast) / vvDt) * 3.6;
    // relativeAltitudeLast = relativeAltitude;
    // static float verticalVelocityLast = verticalVelocity;
    // verticalVelocity = verticalVelocity * 0.05 + verticalVelocityLast * 0.95;
    // verticalVelocityLast = verticalVelocity;

    // static uint32_t vseLUS = microsThisCycle;
    // if (batteryVoltage > 5.0 && micros() - vseLUS > 1000000) {
    //   vseLUS = micros();
    //   static uint16_t ea = 300;
    //   if (ea < 1020 && eeprom_is_ready()) {
    //     int16_t vv = round(verticalVelocity * 100.0f);
    //     eeprom::write(ea, vv);
    //     int16_t ra = round(relativeAltitude * 100.0f);
    //     eeprom::write(ea + 2, ra);
    //     ea += 4;
    //   }
    // }
    // Serial.print("RA: "); Serial.println(relativeAltitude);
    // Serial.print("S: "); Serial.print(verticalSpeed); Serial.println("km/h");

    // static uint32_t lus = 0;
    // if (millis() - lus > 100) {
    //   lus = millis();
    //   Serial.print("P: "); Serial.print(p);
    //   Serial.print("mb\t A: "); Serial.print(absoluteAltitude);
    //   Serial.print("m\t TOA: "); Serial.print(takeOffAltitude);
    //   Serial.print("m\t RA: "); Serial.print(relativeAltitude);
    //   Serial.print("m\t VS: "); Serial.print(verticalSpeed);
    //   Serial.println("km/h");
    // }

    // if (verticalVelocity > recordClimbSpeed) {
    //   recordClimbSpeed = verticalVelocity;
    //   eeprom::write(recordClimbSpeed_address, recordClimbSpeed);
    //   // Serial.print("Speed: "); Serial.println(verticalVelocity);
    // } else if (verticalVelocity < recordFallSpeed) {
    //   recordFallSpeed = verticalVelocity;
    //   eeprom::write(recordFallSpeed_address, recordFallSpeed);
    //   // Serial.print("Speed: "); Serial.println(verticalSpeed);
    // }

    // Serial.print("Alt t: "); Serial.println(micros() - m1);
    // Serial.print(F("T: "); Serial.print(t);
    // Serial.print(F(", P: "); Serial.print(p);
    // Serial.print(F(", RA: "); Serial.println(relativeAltitude);


    static uint32_t batLUS = microsThisCycle;
    if (microsThisCycle - batLUS > 500000 && batteryVoltage > 5.0f
        && microsThisCycle > 1200000000) {
      batLUS = microsThisCycle;
      // uint16_t batt_mv = round((float)batteryVoltage * 1000.0f);
      // uint16_t relativeCurrent = (OCR0B + OCR1B + OCR0A + OCR1A);
      static int16_t ea = 300;
      if (ea < 1010) {
        eeprom_busy_wait();
        // eeprom_write_byte((uint8_t*)address, (uint8_t)value);
        // eeprom::write(ea, w);
        Str str;
        str.b_mv = (float)batteryVoltage * 1000.0f;
        str.rc = (OCR0B + OCR1B + OCR0A + OCR1A);
        eeprom_write_block((const void*)&str, (void*)ea, sizeof(str));
        // eeprom::write(ea, batt_mv);
        // eeprom::write(ea + 2, relativeCurrent);
        ea += 4;
      } else {
        // ea = 300;
      }
    }

    // void  eeprom_read_block (void *__dst, const void *__src, size_t __n)
    // void  eeprom_write_block (const void *__src, void *__dst, size_t __n)
  }

  // static float t = 0.0f;
  // static float p = 0.0f;
  // if (innerCycles == 2) {
  //   bmp.readPressureAndTemperature(&p, &t);
  // } else if (innerCycles == 3) {
  //   float absoluteAltitude = bmp.calculateAltitude(p, 1013, t);
  //   relativeAltitude = absoluteAltitude - takeOffAltitude;

  //   static float relativeAltitudeLastFilter = relativeAltitude;
  //   relativeAltitude = relativeAltitude * 0.01 + relativeAltitudeLastFilter * 0.99;
  //   relativeAltitudeLastFilter = relativeAltitude;
  //   // Serial.println(relativeAltitude);

  //   if (relativeAltitude > recordRelativeAltitude) {
  //     recordRelativeAltitude = relativeAltitude;
  //     EEPROM.put(504, recordRelativeAltitude);
  //   } else if (absoluteAltitude < takeOffAltitude) {
  //     takeOffAltitude = absoluteAltitude;
  //   }

  //   static uint32_t dRelativeAltitudeLUS = 0;
  //   WAIT_FOR_CYCLE(dRelativeAltitudeLUS);
  //   dRelativeAltitudeLUS = micros();
  //   static float relativeAltitudeLast = relativeAltitude;
  //   float verticalSpeed = (relativeAltitude - relativeAltitudeLast) / 0.0031;
  //   relativeAltitudeLast = relativeAltitude;
  //   // Serial.print("S: "); Serial.println(verticalSpeed);

  //   if (verticalSpeed > recordClimbSpeed) {
  //     recordClimbSpeed = verticalSpeed;
  //     EEPROM.put(600, recordClimbSpeed);
  //     Serial.print("Speed: "); Serial.println(verticalSpeed);
  //   } else if (verticalSpeed < recordFallSpeed) {
  //     recordFallSpeed = verticalSpeed;
  //     EEPROM.put(700, recordFallSpeed);
  //     Serial.print("Speed: "); Serial.println(verticalSpeed);
  //   }

  //   // Serial.print("Alt t: "); Serial.println(micros() - m1);
  //   // Serial.print(F("T: "); Serial.print(t);
  //   // Serial.print(F(", P: "); Serial.print(p);
  //   // Serial.print(F(", RA: "); Serial.println(relativeAltitude);
  // }

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
    // DEBUG("Invalid message: "); DEBUG(command.senderId);
    // DEBUG(", "); DEBUG(command.messageId);
    // DEBUG(", "); DEBUG(command.throttle);
    // DEBUG(", "); DEBUG(command.pitch);
    // DEBUG(", "); DEBUG(command.roll);
    // DEBUG(", "); DEBUG(command.yaw);
    // DEBUG(", "); DEBUG((uint8_t)command.flightMode); DEBUGLN();
    OCR0B = 0;
    OCR1B = 0;
    OCR0A = 0;
    OCR1A = 0;
    throttle = 0;
    command.pitch = 0;
    command.roll = 0;
    command.yaw = 0;
    command.flightMode = FlightMode::stabilize;
    altitudeHold = false;
    altitudeHoldSetTime = 0;
    verticalVelocity_pid.off();
  } else {
    if (command.flightMode == FlightMode::stabilize) {
      pid::outer::pitch.setMode(AUTOMATIC);
      pid::outer::roll.setMode(AUTOMATIC);
    } else if (command.flightMode == FlightMode::acro) {
      pid::outer::pitch.setMode(MANUAL);
      pid::outer::roll.setMode(MANUAL);
      pid::inner::setpoint::pitch = command.pitch;
      pid::inner::setpoint::roll = command.roll;
      altitudeHold = false;
    } else {
      pid::inner::pitch.off();
      pid::inner::roll.off();
      pid::outer::pitch.off();
      pid::outer::roll.off();
    }
    altitudeHold = false;
    verticalVelocity_pid.off();
    throttle = command.throttle;
    // if (command.altitudeHold && command.throttle > 5) {
    //   altitudeHold = true;
    //   altitudeHoldSetTime = micros();
    //   verticalVelocity_pid.on();
    //   int16_t vvs = command.throttle - 63;
    //   if (vvs > 10) {
    //     verticalVelocity_setpoint = vvs / 3.0f;
    //   } else if (vvs < - 10) {
    //     verticalVelocity_setpoint = vvs / 3.0f;
    //   } else {
    //     verticalVelocity_setpoint = 0.0f;
    //   }
    // } else {
    //   altitudeHold = false;
    //   altitudeHoldSetTime = 0;
    //   verticalVelocity_pid.off();
    //   throttle = command.throttle;
    // }
    if (command.flightMode == FlightMode::acro) {
      altitudeHold = false;
    }
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
  // case SettingId::imuLpf_common:
  //   if (setting.request) {
  //     setting.value = (float)config::imu::lowPassFilter::common();
  //     DEBUGLN("Requested LPF");
  //   } else {
  //     setting.success = config::imu::lowPassFilter::common.changeValue(setting.value);
  //     setting.value = (float)config::imu::lowPassFilter::common();
  //     imu::mpu6050.setDLPFMode(config::imu::lowPassFilter::common());
  //     DEBUG("LPF changed to "); DEBUGLN(setting.value);
  //   }
  //   break;

  // case SettingId::imuLpfAv_state:
  //   if (setting.request) {
  //     DEBUGLN("Requested LPFav state");
  //     setting.value = (float)config::imu::lowPassFilter::angularVelocity::state();
  //   } else {
  //     setting.success = config::imu::lowPassFilter::angularVelocity::state.changeValue(setting.value);
  //     setting.value = (float)config::imu::lowPassFilter::angularVelocity::state();
  //     DEBUG("LPFav state changed to "); DEBUGLN(setting.value);
  //   }
  //   break;
  // case SettingId::imuLpfAv_alpha:
  //   if (setting.request) {
  //     DEBUGLN("Requested LPFav alpha");
  //     setting.value = config::imu::lowPassFilter::angularVelocity::alpha();
  //   } else {
  //     setting.success = config::imu::lowPassFilter::angularVelocity::alpha.changeValue(setting.value);
  //     setting.value = config::imu::lowPassFilter::angularVelocity::alpha();
  //     config::imu::lowPassFilter::angularVelocity::oneMinusAlpha = 1.0 - setting.value;
  //     DEBUG("imuLpfAv_alpha changed to "); DEBUGLN(setting.value);
  //   }
  //   break;

  // case SettingId::imuLpfAcc_state:
  //   if (setting.request) {
  //     DEBUGLN("Requested LPFacc state");
  //     setting.value = (float)config::imu::lowPassFilter::acceleration::state();
  //   } else {
  //     setting.success = config::imu::lowPassFilter::acceleration::state.changeValue(setting.value);
  //     setting.value = (float)config::imu::lowPassFilter::acceleration::state();
  //     DEBUG("LPFacc state changed to "); DEBUGLN(setting.value);
  //   }
  //   break;
  // case SettingId::imuLpfAcc_alpha:
  //   if (setting.request) {
  //     DEBUGLN("Requested LPFacc alpha");
  //     setting.value = config::imu::lowPassFilter::acceleration::alpha();
  //   } else {
  //     setting.success = config::imu::lowPassFilter::acceleration::alpha.changeValue(setting.value);
  //     setting.value = config::imu::lowPassFilter::acceleration::alpha();
  //     config::imu::lowPassFilter::acceleration::oneMinusAlpha = 1.0 - setting.value;
  //     DEBUG("LPFacc alpha changed to "); DEBUGLN(setting.value);
  //   }
  //   break;

  // case SettingId::imuComplementary_alpha:
  //   if (setting.request) {
  //     DEBUGLN("Requested Complementary alpha");
  //     setting.value = config::imu::complementary::alpha();
  //   } else {
  //     setting.success = config::imu::complementary::alpha.changeValue(setting.value);
  //     setting.value = config::imu::complementary::alpha();
  //     config::imu::complementary::oneMinusAlpha = 1.0 - setting.value;
  //     DEBUG("Complementary alpha changed to "); DEBUGLN(setting.value);
  //   }
  //   break;

  // case SettingId::comm_pa:
  //   if (setting.request) {
  //     DEBUGLN("Requested Pwr amp (comm)");
  //     setting.value = (float)config::communication::powerAmplification();
  //   } else {
  //     setting.success = config::communication::powerAmplification.changeValue(setting.value);
  //     setting.value = (float)config::communication::powerAmplification();
  //     comm::rf24.setPALevel(config::communication::powerAmplification());
  //     DEBUG("Pwr amp (comm) changed to "); DEBUGLN(setting.value);
  //   }
  // case SettingId::comm_dataRate:
  //   if (setting.request) {
  //     DEBUGLN("Requested Data rate (comm)");
  //     setting.value = (float)config::communication::dataRate();
  //   } else {
  //     setting.success = config::communication::dataRate.changeValue(setting.value);
  //     setting.value = (float)config::communication::dataRate();
  //     comm::rf24.setDataRate(config::communication::dataRate());
  //     DEBUG("Data rate (comm) changed to "); DEBUGLN(setting.value);
  //   }
  // case SettingId::comm_retryDelay:
  //   if (setting.request) {
  //     DEBUGLN("Requested Retry delay (comm)");
  //     setting.value = (float)config::communication::retryDelay();
  //   } else {
  //     setting.success = config::communication::retryDelay.changeValue(setting.value);
  //     setting.value = (float)config::communication::retryDelay();
  //     comm::rf24.setRetries(config::communication::retryDelay(),
  //                           config::communication::retryCount());
  //     DEBUG("Retry delay (comm) changed to "); DEBUGLN(setting.value);
  //   }
  // case SettingId::comm_retryCount:
  //   if (setting.request) {
  //     DEBUGLN("Requested Retry count (comm)");
  //     setting.value = (float)config::communication::retryCount();
  //   } else {
  //     setting.success = config::communication::retryCount.changeValue(setting.value);
  //     setting.value = (float)config::communication::retryCount();
  //     comm::rf24.setRetries(config::communication::retryDelay(),
  //                           config::communication::retryCount());
  //     DEBUG("Retry count (comm) changed to "); DEBUGLN(setting.value);
  //   }
  //   break;
  // case SettingId::comm_crcLength:
  //   if (setting.request) {
  //     DEBUGLN("Requested CRC length (comm)");
  //     setting.value = (float)config::communication::crcLength();
  //   } else {
  //     setting.success = config::communication::crcLength.changeValue(setting.value);
  //     setting.value = (float)config::communication::crcLength();
  //     comm::rf24.setCRCLength(config::communication::crcLength());
  //     DEBUG("CRC length (comm) changed to "); DEBUGLN(setting.value);
  //   }
  //   break;
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
      setting.value = config::regulation::inner::P();
    } else {
      setting.success = config::regulation::inner::P.changeValue(setting.value);
      setting.value = config::regulation::inner::P();
      pid::inner::pitch.setTunings(setting.value);
      pid::inner::roll.setTunings(setting.value);
      DEBUG("Pi>"); DEBUGLN(setting.value);
    }
    break;
  case SettingId::regInner_yawP:
    if (setting.request) {
      DEBUGLN("?Pi_yaw");
      setting.value = config::regulation::inner::yawP();
    } else {
      setting.success = config::regulation::inner::yawP.changeValue(setting.value);
      setting.value = config::regulation::inner::yawP();
      pid::inner::yaw.setTunings(setting.value);
      DEBUG("Pi_yaw>"); DEBUGLN(setting.value);
    }
    break;
  // case SettingId::regInner_outputLimit:
  //   if (setting.request) {
  //     DEBUGLN("Requested Output limit (inner)");
  //     setting.value = (float)config::regulation::inner::outputLimit();
  //   } else {
  //     setting.success = config::regulation::inner::outputLimit.changeValue(setting.value);
  //     setting.value = (float)config::regulation::inner::outputLimit();
  //     pid::inner::pitch.setOutputLimits(config::regulation::inner::outputLimit());
  //     pid::inner::roll.setOutputLimits(config::regulation::inner::outputLimit());
  //     pid::inner::yaw.setOutputLimits(config::regulation::inner::outputLimit());
  //     DEBUG("Output limit (inner) changed to "); DEBUGLN(setting.value);
  //   }
  //   break;
  case SettingId::regOuter_p:
    if (setting.request) {
      DEBUGLN("?Po");
      setting.value = config::regulation::outer::P();
    } else {
      setting.success = config::regulation::outer::P.changeValue(setting.value);
      setting.value = config::regulation::outer::P();
      pid::outer::pitch.setTunings(setting.value, config::regulation::outer::I());
      pid::outer::roll.setTunings(setting.value, config::regulation::outer::I());
      DEBUG("Po>"); DEBUGLN(setting.value);
    }
    break;
  case SettingId::regOuter_i:
    if (setting.request) {
      DEBUGLN("?Io");
      setting.value = config::regulation::outer::I();
    } else {
      setting.success = config::regulation::outer::I.changeValue(setting.value);
      setting.value = config::regulation::outer::I();
      pid::outer::pitch.setTunings(config::regulation::outer::P(), setting.value);
      pid::outer::roll.setTunings(config::regulation::outer::P(), setting.value);
      DEBUG("Io>"); DEBUGLN(setting.value);
    }
    break;
  // case SettingId::regOuter_updateRate:
  //   if (setting.request) {
  //     DEBUGLN("Requested Outer update rate");
  //     setting.value = (float)config::regulation::outer::updateRate();
  //   } else {
  //     setting.success = config::regulation::outer::updateRate.changeValue(setting.value);
  //     setting.value = (float)config::regulation::outer::updateRate();
  //     pid::outer::pitch.setUpdateRate(config::regulation::outer::updateRate());
  //     pid::outer::roll.setUpdateRate(config::regulation::outer::updateRate());
  //     DEBUG("Outer update rate changed to "); DEBUGLN(setting.value);
  //   }
  //   break;
  // case SettingId::regOuter_outputLimit:
  //   if (setting.request) {
  //     DEBUGLN("Requested Outer output limit");
  //     setting.value = (float)config::regulation::outer::outputLimit();
  //   } else {
  //     setting.success = config::regulation::outer::outputLimit.changeValue(setting.value);
  //     setting.value = (float)config::regulation::outer::outputLimit();
  //     pid::inner::pitch.setOutputLimits(config::regulation::outer::outputLimit());
  //     pid::inner::roll.setOutputLimits(config::regulation::outer::outputLimit());
  //     pid::inner::yaw.setOutputLimits(config::regulation::outer::outputLimit());
  //     DEBUG("Outer output limit changed to "); DEBUGLN(setting.value);
  //   }
  //   break;
  // case SettingId::reg_minRegThrottle:
  //   if (setting.request) {
  //     DEBUGLN("Requested Min reg throttle");
  //     setting.value = (float)config::regulation::minimumRegulationThrottle();
  //   } else {
  //     setting.success = config::regulation::minimumRegulationThrottle.changeValue(setting.value);
  //     setting.value = (float)config::regulation::minimumRegulationThrottle();
  //     DEBUG("Min reg throttle changed to "); DEBUGLN(setting.value);
  //   }
  //   break;
  // case SettingId::reg_maxBaseThrottle:
  //   if (setting.request) {
  //     DEBUGLN("Requested Max base throttle");
  //     setting.value = (float)config::regulation::maximumBaseThrottle();
  //   } else {
  //     setting.success = config::regulation::maximumBaseThrottle.changeValue(setting.value);
  //     setting.value = (float)config::regulation::maximumBaseThrottle();
  //     DEBUG("Max base throttle changed to "); DEBUGLN(setting.value);
  //   }
  //   break;

  case SettingId::indication_armsLevel:
    if (setting.request) {
      DEBUGLN("?Arms lvl");
      setting.value = (float)config::indication::armsLevel();
    } else {
      setting.success = config::indication::armsLevel.changeValue(setting.value);
      setting.value = (float)config::indication::armsLevel();
      // indication::arms(expMap[clamp(config::indication::armsLevel(), 0, 13)]);
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
  case SettingId::regAlt_p:
    if (setting.request) {
      DEBUGLN("?AltP");
      setting.value = config::regulation::altitude::P();
    } else {
      setting.success = config::regulation::altitude::P.changeValue(setting.value);
      setting.value = config::regulation::altitude::P();
      verticalVelocity_pid.setTunings(setting.value, config::regulation::altitude::P(), 0.0f, config::regulation::altitude::D());
      DEBUG("AltP>"); DEBUGLN(setting.value);
    }
    break;
  case SettingId::regAlt_d:
    if (setting.request) {
      DEBUGLN("?AltD");
      setting.value = config::regulation::altitude::D();
    } else {
      setting.success = config::regulation::altitude::D.changeValue(setting.value);
      setting.value = config::regulation::altitude::D();
      verticalVelocity_pid.setTunings(setting.value, config::regulation::altitude::P(), 0.0f, config::regulation::altitude::D());
      DEBUG("AltD>"); DEBUGLN(setting.value);
    }
  // case SettingId::imuCalibrate:
  //   // calibrate
  //   // DEBUG("Cal>\n");
  //   break;
  default:
    setting.success = false;
    DEBUG("Unrec sId: "); DEBUGLN((uint8_t)setting.id);
    break;
  } //switch (setting.id)
  comm::rf24.setResponse(&setting, sizeof(setting));
}

