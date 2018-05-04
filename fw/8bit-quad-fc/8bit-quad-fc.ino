
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#include "src/config/config.hpp"
#include "src/util/debug.hpp"
#include "src/board/8bit-quad-fc_board.hpp"
#include "src/peripheral/ADC.hpp"
#include "src/imu/MPU6050.hpp"
#include "src/control/PID.hpp"
#include "src/radio/nRF24L01p.hpp"
#include "src/util/utils.hpp"

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

namespace imu {
MPU6050_i2cDev mpu6050(config::imu::i2cAddress);
} //namespace imu
AngularVelocity angularVelocity;
Acceleration acceleration;
Attitude attitude;


namespace comm {
nRF24L01p_RF24 rf24(Role::drone, pin::communication::ino::ce, pin::communication::ino::csn);
} //namespace comm
Command command;


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
Pcontroller pitch(&setpoint::pitch, &angularVelocity.y, &output::pitch);
Pcontroller roll(&setpoint::roll, &angularVelocity.x, &output::roll);
Pcontroller yaw(&command.yaw, &angularVelocity.z, &output::yaw);
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
float gravityAcceleration = 0.0;


void setup() {
  bool success = true;

  replaceTimer0WithTimer2();

  initIO();
  indication::signal(OFF);
  indication::warning(ON);
  indication::arms(OFF);

  INIT_UART(config::debug::baud);
  Serial.begin(2000000);
  DEBUG(F("FC2.\n"));

  config::init();
#if DEBUG_SETTINGS
  DEBUG(F("config::imu::lowPassFilter::common: ")); DEBUGLN(config::imu::lowPassFilter::common());
  DEBUG(F("config::imu::lowPassFilter::angularVelocity::state: ")); DEBUGLN(config::imu::lowPassFilter::angularVelocity::state());
  DEBUG(F("config::imu::lowPassFilter::angularVelocity::alpha: ")); DEBUGLN(config::imu::lowPassFilter::angularVelocity::alpha());
  DEBUG(F("config::imu::lowPassFilter::angularVelocity::oneMinusAlpha")); DEBUGLN(config::imu::lowPassFilter::angularVelocity::oneMinusAlpha);
  DEBUG(F("config::imu::lowPassFilter::acceleration::state: ")); DEBUGLN(config::imu::lowPassFilter::acceleration::state());
  DEBUG(F("config::imu::lowPassFilter::acceleration::alpha: ")); DEBUGLN(config::imu::lowPassFilter::acceleration::alpha());
  DEBUG(F("config::imu::lowPassFilter::acceleration::oneMinusAlpha")); DEBUGLN(config::imu::lowPassFilter::acceleration::oneMinusAlpha);
  DEBUG(F("config::imu::complementary::alpha: ")); DEBUGLN(config::imu::complementary::alpha());
  DEBUG(F("config::imu::complementary::oneMinusAlpha")); DEBUGLN(config::imu::complementary::oneMinusAlpha);

  DEBUG(F("config::communication::powerAmplification: ")); DEBUGLN(config::communication::powerAmplification());
  DEBUG(F("config::communication::dataRate: ")); DEBUGLN(config::communication::dataRate());
  DEBUG(F("config::communication::retryDelay: ")); DEBUGLN(config::communication::retryDelay());
  DEBUG(F("config::communication::retryCount: ")); DEBUGLN(config::communication::retryCount());
  DEBUG(F("config::communication::crcLength: ")); DEBUGLN(config::communication::crcLength());
  DEBUG(F("config::communication::telemetry::type: ")); DEBUGLN(config::communication::telemetry::type());

  DEBUG(F("config::regulation::inner::P: ")); DEBUGLN(config::regulation::inner::P());
  DEBUG(F("config::regulation::inner::yawP: ")); DEBUGLN(config::regulation::inner::yawP());
  DEBUG(F("config::regulation::inner::outputLimit: ")); DEBUGLN(config::regulation::inner::outputLimit());
  DEBUG(F("config::regulation::outer::P: ")); DEBUGLN(config::regulation::outer::P());
  DEBUG(F("config::regulation::outer::I: ")); DEBUGLN(config::regulation::outer::I());
  DEBUG(F("config::regulation::outer::updateRate: ")); DEBUGLN(config::regulation::outer::updateRate());
  DEBUG(F("config::regulation::outer::outputLimit: ")); DEBUGLN(config::regulation::outer::outputLimit());
  DEBUG(F("config::regulation::minimumRegulationThrottle: ")); DEBUGLN(config::regulation::minimumRegulationThrottle());
  DEBUG(F("config::regulation::maximumBaseThrottle: ")); DEBUGLN(config::regulation::maximumBaseThrottle());

  DEBUG(F("config::indication::armsLevel: ")); DEBUGLN(config::indication::armsLevel());
  DEBUG(F("config::indication::lamp: ")); DEBUGLN(config::indication::lamp());
  DEBUGLN();
#endif

  initADC();

  // IMU initialization
  success &= imu::mpu6050.initialize();
  imu::mpu6050.setDLPFMode(config::imu::lowPassFilter::common());
  DEBUGLN(F("Initialized IMU"));
  // ~IMU initialization -----

  // Communication initialization
  comm::rf24.initialize();
  comm::rf24.setPALevel(config::communication::powerAmplification());
  comm::rf24.setDataRate(config::communication::dataRate());
  comm::rf24.setRetries(config::communication::retryDelay(), config::communication::retryCount());
  comm::rf24.setCRCLength(config::communication::crcLength());
  // ~Communication initialization -----

  // Regulation initialization
  success &= pid::inner::pitch.setTunings(config::regulation::inner::P());
  success &= pid::inner::roll.setTunings(config::regulation::inner::P());
  success &= pid::inner::yaw.setTunings(config::regulation::inner::yawP());
  success &= pid::inner::pitch.setOutputLimits(config::regulation::inner::outputLimit());
  success &= pid::inner::roll.setOutputLimits(config::regulation::inner::outputLimit());
  success &= pid::inner::yaw.setOutputLimits(config::regulation::inner::outputLimit());
  pid::inner::pitch.on(); pid::inner::roll.on(); pid::inner::yaw.on();

  success &= pid::outer::pitch.setTunings(config::regulation::outer::P(), config::regulation::outer::I());
  success &= pid::outer::roll.setTunings(config::regulation::outer::P(), config::regulation::outer::I());
  pid::outer::pitch.setUpdateRate(config::regulation::outer::updateRate());
  pid::outer::roll.setUpdateRate(config::regulation::outer::updateRate());
  success &= pid::outer::pitch.setOutputLimits(config::regulation::outer::outputLimit());
  success &= pid::outer::roll.setOutputLimits(config::regulation::outer::outputLimit());
  pid::outer::pitch.on(); pid::outer::roll.on();
  // ~Regulation initialization -----


  for (auto i = 0; i < 5; ++i) {
    Acceleration accel;
    imu::mpu6050.getAcceleration(&accel);
  }
  int32_t zRaw = 0;
  for (auto i = 0; i < 50; ++i) {
    Acceleration accel;
    imu::mpu6050.getAcceleration(&accel);
    zRaw += accel.z;
  }
  zRaw /= 50;
  gravityAcceleration = imu::mpu6050.toGForce(zRaw);


  if (!success) { DEBUG(F("Wrong initialization configuration!")); exit(0); }
  indication::signal(ON);
  indication::warning(OFF);
  indication::lamp(config::indication::lamp());
  indication::arms(config::indication::armsLevel());


  pinMode(INTERRUPT_PIN, INPUT);
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(imu::mpu6050._mpu6050.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = imu::mpu6050._mpu6050.dmpInitialize();
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    imu::mpu6050._mpu6050.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = imu::mpu6050._mpu6050.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = imu::mpu6050._mpu6050.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  DEBUGLN(F("Setup done."));
} //void setup()


void loop() {
  static uint32_t microsThisCycle = micros();
  static uint32_t microsNow = microsThisCycle;
  microsThisCycle = micros();

#if DEBUG_LOOP_TIME
  static uint32_t loopTimeLUS = microsThisCycle;
  DEBUG("Loop time: "); DEBUG((microsThisCycle - loopTimeLUS)); DEBUG("\n");
  loopTimeLUS = microsThisCycle;
  // indication::toggleLamp();
#endif


  // Regulation
  // static uint32_t innerLUS = microsNow;
  // if (microsNow > innerLUS + pid::inner::yaw.getUpdateRate() - 1000) {
  static uint32_t innerComputeLUS = microsThisCycle;
  while (micros() - innerComputeLUS < 1996);
  pid::inner::yaw.compute();
  pid::inner::pitch.compute();
  pid::inner::roll.compute();
  innerComputeLUS = micros();
  static int8_t innerCycles = 0;
  ++innerCycles;
  // innerLUS = microsNow;
  // }
  // microsNow = micros();
  // static uint32_t outerLUS = microsNow;
  // if (microsNow > outerLUS + pid::outer::roll.getUpdateRate() - config::regulation::outer::updateRateTolerance) {
  if (innerCycles >= 4 || innerCycles < 0) {
    innerCycles = 0;
    pid::outer::pitch.compute();
    pid::outer::roll.compute();
    // outerLUS = microsNow;
  }
  // }

#if DEBUG_PID_PITCH
  if (innerCycles % 3) {
    if (command.flightMode == FlightMode::stabilize) {
      DEBUG(command.pitch);
      DEBUG(","); DEBUG(attitude.pitch);
    } else {
      DEBUG(","); DEBUG(pid::inner::setpoint::pitch);
      DEBUG(","); DEBUG(angularVelocity.y);
    }
    DEBUG(","); DEBUGLN(pid::inner::output::pitch);
  }
#endif
#if DEBUG_PID_ROLL
  if (innerCycles % 3) {
    if (command.flightMode == FlightMode::stabilize) {
      DEBUG(command.roll);
      DEBUG(","); DEBUG(attitude.roll);
    }
    DEBUG(","); DEBUG(pid::inner::setpoint::roll);
    DEBUG(","); DEBUG(angularVelocity.x);
    DEBUG(","); DEBUGLN(pid::inner::output::roll);
  }
#endif
  // ~Regulation -----


  static float droll = 0.0;
  static float dpitch = 0.0;
  if (!dmpReady) Serial.println("DMP ERROR");

  mpuIntStatus = imu::mpu6050._mpu6050.getIntStatus();
  fifoCount = imu::mpu6050._mpu6050.getFIFOCount();
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    imu::mpu6050._mpu6050.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = imu::mpu6050._mpu6050.getFIFOCount();
    // read a packet from FIFO
    imu::mpu6050._mpu6050.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    imu::mpu6050._mpu6050.dmpGetQuaternion(&q, fifoBuffer);
    imu::mpu6050._mpu6050.dmpGetGravity(&gravity, &q);
    imu::mpu6050._mpu6050.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // Serial.print("ypr\t");
    // Serial.print(ypr[0] * 180 / M_PI);
    // Serial.print("\t");
    // Serial.print(ypr[1] * 180 / M_PI);
    // Serial.print("\t");
    // Serial.println(ypr[2] * 180 / M_PI);
    droll = ypr[1] * 180 / M_PI;
  Serial.print(droll);
  Serial.print(",");
  Serial.println(attitude.roll);
  }


  // Attitude -----
  // Fetch
  // uint32_t time = micros();
  imu::mpu6050.getMotion(&angularVelocity, &acceleration); // 1900us, 770 @ 400kHz, 600 @ 1MHz
  // DEBUG("Time: "); DEBUGLN(micros() - time);

  // Low-pass filter
  // Angular velocity
  // if (config::imu::lowPassFilter::angularVelocity::state() == true) {
    static AngularVelocity angularVelocity_last;
    angularVelocity.y = angularVelocity.y * config::imu::lowPassFilter::angularVelocity::alpha()
                        + angularVelocity_last.y * config::imu::lowPassFilter::angularVelocity::oneMinusAlpha;
    angularVelocity.x = angularVelocity.x * config::imu::lowPassFilter::angularVelocity::alpha()
                        + angularVelocity_last.x * config::imu::lowPassFilter::angularVelocity::oneMinusAlpha;
    angularVelocity.z = angularVelocity.z * config::imu::lowPassFilter::angularVelocity::alpha()
                        + angularVelocity_last.z * config::imu::lowPassFilter::angularVelocity::oneMinusAlpha;
    angularVelocity_last.y = angularVelocity.y;
    angularVelocity_last.x = angularVelocity.x;
    angularVelocity_last.z = angularVelocity.z;
  // }
  // Acceleration
  // if (config::imu::lowPassFilter::acceleration::state() == true) {
    static Acceleration acceleration_last;
    acceleration.y = acceleration.y * config::imu::lowPassFilter::acceleration::alpha()
                     + acceleration_last.y * config::imu::lowPassFilter::acceleration::oneMinusAlpha;
    acceleration.x = acceleration.x * config::imu::lowPassFilter::acceleration::alpha()
                     + acceleration_last.x * config::imu::lowPassFilter::acceleration::oneMinusAlpha;
    acceleration.z = acceleration.z * config::imu::lowPassFilter::acceleration::alpha()
                     + acceleration_last.z * config::imu::lowPassFilter::acceleration::oneMinusAlpha;
    acceleration_last.y = acceleration.y;
    acceleration_last.x = acceleration.x;
    acceleration_last.z = acceleration.z;
  // }

#if DEBUG_ANGULAR_VELOCITY
  if (innerCycles == 0) {
    DEBUG(angularVelocity.x); DEBUG(F(", "));
    DEBUG(angularVelocity.y); DEBUG(F(", "));
    DEBUG(angularVelocity.z); DEBUGLN();
  }
#endif
#if DEBUG_ACCELERATION
  if (innerCycles == 0) {
    DEBUG(acceleration.x); DEBUG(F(", "));
    DEBUG(acceleration.y); DEBUG(F(", "));
    DEBUG(acceleration.z); DEBUGLN();
  }
#endif


  // Accelerometer angle
  float x_sqr = square((float)acceleration.x);
  float y_sqr = square((float)acceleration.y);
  float z_sqr = square((float)acceleration.z);

  float pitch_a = atan2( (float) - acceleration.x,
                         (sign((float)acceleration.z) * sqrt(z_sqr + (y_sqr * config::imu::epsilon))) ) * toDegrees;
  float roll_a = atan2( (float)acceleration.y,
                        sqrt(x_sqr + z_sqr) ) * toDegrees;

#if DEBUG_ACCELEROMETER_ANGLE
  if (innerCycles == 0) {
    DEBUG(pitch_a); DEBUG(", "); DEBUG(angularVelocity.x);
    DEBUG(", ");  DEBUG(roll_a); DEBUG(", "); DEBUGLN(angularVelocity.y);
  }
#endif

  // Fusion
  static uint32_t fusionLUS = microsThisCycle;
  microsNow = micros();
  float dt = (float)(microsNow - fusionLUS) / 1000000.0;

  /*
    float pitchNow_g = attitude.pitch + angularVelocity.y * dt;
    float pitch_g = pitchNow_g + ((abs(pitchNow_g - attitude.pitch) / 2) * dt);
    float rollNow_g = attitude.roll + angularVelocity.x * dt;
    float roll_g = rollNow_g + ((abs(rollNow_g - attitude.roll) / 2) * dt);

    attitude.pitch = pitch_g * config::imu::complementary::alpha()
                     + pitch_a * config::imu::complementary::oneMinusAlpha;
    attitude.roll = roll_g * config::imu::complementary::alpha()
                   + roll_a * config::imu::complementary::oneMinusAlpha;
  */

  attitude.pitch = (attitude.pitch + angularVelocity.y * dt) * config::imu::complementary::alpha()
                   + pitch_a * config::imu::complementary::oneMinusAlpha;
  attitude.roll = (attitude.roll + angularVelocity.x * dt) * config::imu::complementary::alpha()
                  + roll_a * config::imu::complementary::oneMinusAlpha;

  fusionLUS = microsNow;

#if DEBUG_ATTITUDE
  if (innerCycles == 0) {
    DEBUG(attitude.pitch); DEBUG(F(", ")); DEBUG(attitude.roll);
    DEBUGLN();
  }
#endif
  // ~Attitude -----


  // // Z acceleration
  // float z_a = acceleration.z * 0.00119710083;
  // static float zLast = 0;
  // float z = z_a * 0.7 + zLast * 0.3;
  // zLast = z;
  // float zLinear = z - gravityAcceleration;
  // static uint32_t zLinDispLUS = microsThisCycle;
  // if (microsThisCycle > zLinDispLUS + 4000) {
  //   zLinDispLUS = microsThisCycle;
  //   // if (zLinear > -1.5 && zLinear < 1.5) zLinear = 0.0;
  //   if (zLinear > 1.5) Serial.println("UUUUP");
  //   else if (zLinear < -1.5) Serial.println("down");
  //   // else Serial.println("Hovering");
  //   // Serial.print(z); Serial.print(", "); Serial.println(zLinear);
  // }
  // // ~Z acceleration


  // Motor mix
  // if (newComputation == true) {
  uint8_t tl_p = 0;
  uint8_t tr_p = 0;
  uint8_t bl_p = 0;
  uint8_t br_p = 0;
  if (command.throttle > config::regulation::minimumRegulationThrottle()) {
    if (command.throttle > config::regulation::maximumBaseThrottle()) command.throttle = config::regulation::maximumBaseThrottle();
    tl_p = clamp(command.throttle
                 - pid::inner::output::pitch - pid::inner::output::roll - pid::inner::output::yaw,
                 0, 127);
    tr_p = clamp(command.throttle
                 - pid::inner::output::pitch + pid::inner::output::roll + pid::inner::output::yaw,
                 0, 127);
    bl_p = clamp(command.throttle
                 + pid::inner::output::pitch - pid::inner::output::roll + pid::inner::output::yaw,
                 0, 127);
    br_p = clamp(command.throttle
                 + pid::inner::output::pitch + pid::inner::output::roll - pid::inner::output::yaw,
                 0, 127);
  } else {
    tl_p = command.throttle;
    tr_p = command.throttle;
    bl_p = command.throttle;
    br_p = command.throttle;
    pid::outer::pitch.unwind();
    pid::outer::roll.unwind();
  }
  if (command.flightMode == FlightMode::direct) {
    tl_p = clamp(command.throttle
                 - command.pitch - command.roll - command.yaw,
                 0, 127);
    tr_p = clamp(command.throttle
                 - command.pitch + command.roll + command.yaw,
                 0, 127);
    bl_p = clamp(command.throttle
                 + command.pitch - command.roll + command.yaw,
                 0, 127);
    br_p = clamp(command.throttle
                 + command.pitch + command.roll - command.yaw,
                 0, 127);
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
      DEBUG(OCR0B); DEBUG("\t"); DEBUGLN(OCR1B);
      DEBUG(OCR0A); DEBUG("\t"); DEBUGLN(OCR1A);
      uint8_t i = 0;
      while (i++ < 12) DEBUGLN();
    }
  }
#endif
  // ~Motor mix -----


  // Communication
  static uint32_t lastCommandLUS = microsThisCycle;
  static uint32_t batteryVoltageLUS = microsThisCycle;
  if (comm::rf24.available()) {
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
      case PacketType::Telemetry_imu:
        DEBUGLN(F("shouldn't receive telemetry_imu"));
        break;
      case PacketType::Telemetry_motors:
        DEBUGLN(F("shouldn't receive telemetry_motors"));
        break;
      default:
        DEBUG(F("unrecognized ")); DEBUGLN(rawMessage[0]);
        break;
      }
    }
  } else if (microsThisCycle > lastCommandLUS + config::communication::commandTimeout) {
    command.throttle = 0;
    pid::outer::pitch.on();
    pid::outer::roll.on();
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
  } else if (microsThisCycle > batteryVoltageLUS + config::battery::updateRate && innerCycles != 0) {
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


// Indication
  static uint32_t indicationLUS = microsThisCycle;
  if (microsThisCycle > indicationLUS + config::indication::period && innerCycles != 0) {
    indicationLUS = microsThisCycle;
    if (status::battery == Status::normal && status::communication == Status::normal) {
      indication::toggleSignal();
      indication::warning(0);
      indication::arms(config::indication::armsLevel());
    } else if (status::communication == Status::error || status::communication == Status::warning) {
      indication::toggleSignal();
      indication::warning(!indication::signal());
      indication::toggleArms();
    } else if (status::battery == Status::warning) {
      indication::toggleSignal();
      indication::warning(!indication::signal());
      indication::arms(config::indication::armsLevel());
    } else if (status::battery == Status::error) {
      indication::signal(0);
      indication::toggleWarning();
      indication::arms(config::indication::armsLevel());
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
    comm::rf24.setResponse(0, 1);
    break;
  case 1:
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
  case 2:
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
  if (command.senderId != senderId) {
    DEBUG(F("Invalid sender ")); DEBUGLN(command.senderId);
  } else {
    if (command.flightMode == FlightMode::stabilize) {
      pid::outer::pitch.setMode(AUTOMATIC);
      pid::outer::roll.setMode(AUTOMATIC);
    } else if (command.flightMode == FlightMode::acro) {
      pid::outer::pitch.setMode(MANUAL);
      pid::outer::roll.setMode(MANUAL);
      pid::inner::setpoint::pitch = command.pitch;
      pid::inner::setpoint::roll = command.roll;
    } else {
      pid::outer::pitch.off();
      pid::outer::roll.off();
      pid::outer::pitch.off();
      pid::outer::roll.off();
    }
    status::communication = Status::normal;
  }
}

void handleSetting(Setting & setting) {
  setting.success = true;
  switch (setting.id) {
  case SettingId::dummy:
    setting.success = false;
    DEBUGLN(F("dummy"));
    break;
  case SettingId::imuLpf_common:
    if (setting.request) {
      setting.value = (float)config::imu::lowPassFilter::common();
      DEBUGLN(F("Requested imuLpf_common"));
    } else {
      setting.success = config::imu::lowPassFilter::common.changeValue(setting.value);
      setting.value = (float)config::imu::lowPassFilter::common();
      imu::mpu6050.setDLPFMode(config::imu::lowPassFilter::common());
      DEBUG(F("imuLpf_common changed to ")); DEBUGLN(setting.value);
    }
    break;

  case SettingId::imuLpfAv_state:
    if (setting.request) {
      DEBUGLN(F("Requested "));
      setting.value = (float)config::imu::lowPassFilter::angularVelocity::state();
    } else {
      setting.success = config::imu::lowPassFilter::angularVelocity::state.changeValue(setting.value);
      setting.value = (float)config::imu::lowPassFilter::angularVelocity::state();
      DEBUG(F(" changed to ")); DEBUGLN(setting.value);
    }
    break;
  case SettingId::imuLpfAv_alpha:
    if (setting.request) {
      DEBUGLN(F("Requested imuLpfAv_alpha"));
      setting.value = config::imu::lowPassFilter::angularVelocity::alpha();
    } else {
      setting.success = config::imu::lowPassFilter::angularVelocity::alpha.changeValue(setting.value);
      setting.value = config::imu::lowPassFilter::angularVelocity::alpha();
      config::imu::lowPassFilter::angularVelocity::oneMinusAlpha = 1.0 - setting.value;
      DEBUG(F("imuLpfAv_alpha changed to ")); DEBUGLN(setting.value);
    }
    break;

  case SettingId::imuLpfAcc_state:
    if (setting.request) {
      DEBUGLN(F("Requested imuLpfAcc_state"));
      setting.value = (float)config::imu::lowPassFilter::acceleration::state();
    } else {
      setting.success = config::imu::lowPassFilter::acceleration::state.changeValue(setting.value);
      setting.value = (float)config::imu::lowPassFilter::acceleration::state();
      DEBUG(F("imuLpfAcc_state changed to ")); DEBUGLN(setting.value);
    }
    break;
  case SettingId::imuLpfAcc_alpha:
    if (setting.request) {
      DEBUGLN(F("Requested imuLpfAcc_alpha"));
      setting.value = config::imu::lowPassFilter::acceleration::alpha();
    } else {
      setting.success = config::imu::lowPassFilter::acceleration::alpha.changeValue(setting.value);
      setting.value = config::imu::lowPassFilter::acceleration::alpha();
      config::imu::lowPassFilter::acceleration::oneMinusAlpha = 1.0 - setting.value;
      DEBUG(F("imuLpfAcc_alpha changed to ")); DEBUGLN(setting.value);
    }
    break;

  case SettingId::imuComplementary_alpha:
    if (setting.request) {
      DEBUGLN(F("Requested imuComplementary_alpha"));
      setting.value = config::imu::complementary::alpha();
    } else {
      setting.success = config::imu::complementary::alpha.changeValue(setting.value);
      setting.value = config::imu::complementary::alpha();
      config::imu::complementary::oneMinusAlpha = 1.0 - setting.value;
      DEBUG(F("imuComplementary_alpha changed to ")); DEBUGLN(setting.value);
    }
    break;

  case SettingId::comm_pa:
    if (setting.request) {
      DEBUGLN(F("Requested comm_pa"));
      setting.value = (float)config::communication::powerAmplification();
    } else {
      setting.success = config::communication::powerAmplification.changeValue(setting.value);
      setting.value = (float)config::communication::powerAmplification();
      comm::rf24.setPALevel(config::communication::powerAmplification());
      DEBUG(F("comm_pa changed to ")); DEBUGLN(setting.value);
    }
  case SettingId::comm_dataRate:
    if (setting.request) {
      DEBUGLN(F("Requested comm_dataRate"));
      setting.value = (float)config::communication::dataRate();
    } else {
      setting.success = config::communication::dataRate.changeValue(setting.value);
      setting.value = (float)config::communication::dataRate();
      comm::rf24.setDataRate(config::communication::dataRate());
      DEBUG(F("comm_dataRate changed to ")); DEBUGLN(setting.value);
    }
  case SettingId::comm_retryDelay:
    if (setting.request) {
      DEBUGLN(F("Requested comm_retryDelay"));
      setting.value = (float)config::communication::retryDelay();
    } else {
      setting.success = config::communication::retryDelay.changeValue(setting.value);
      setting.value = (float)config::communication::retryDelay();
      comm::rf24.setRetries(config::communication::retryDelay(),
                            config::communication::retryCount());
      DEBUG(F("comm_retryDelay changed to ")); DEBUGLN(setting.value);
    }
  case SettingId::comm_retryCount:
    if (setting.request) {
      DEBUGLN(F("Requested comm_retryCount"));
      setting.value = (float)config::communication::retryCount();
    } else {
      setting.success = config::communication::retryCount.changeValue(setting.value);
      setting.value = (float)config::communication::retryCount();
      comm::rf24.setRetries(config::communication::retryDelay(),
                            config::communication::retryCount());
      DEBUG(F("comm_retryCount changed to ")); DEBUGLN(setting.value);
    }
    break;
  case SettingId::comm_crcLength:
    if (setting.request) {
      DEBUGLN(F("Requested comm_crcLength"));
      setting.value = (float)config::communication::crcLength();
    } else {
      setting.success = config::communication::crcLength.changeValue(setting.value);
      setting.value = (float)config::communication::crcLength();
      comm::rf24.setCRCLength(config::communication::crcLength());
      DEBUG(F("comm_crcLength changed to ")); DEBUGLN(setting.value);
    }
    break;
  case SettingId::commTelemetry_type:
    if (setting.request) {
      DEBUGLN(F("Requested commTelemetry_type"));
      setting.value = (float)config::communication::telemetry::type();
    } else {
      setting.success = config::communication::telemetry::type.changeValue(setting.value);
      setting.value = (float)config::communication::telemetry::type();
      DEBUG(F("commTelemetry_type changed to ")); DEBUGLN(setting.value);
    }
    break;

  case SettingId::regInner_p:
    if (setting.request) {
      DEBUGLN(F("Requested regInner_p"));
      setting.value = config::regulation::inner::P();
    } else {
      setting.success = config::regulation::inner::P.changeValue(setting.value);
      setting.value = config::regulation::inner::P();
      pid::inner::pitch.setTunings(setting.value);
      pid::inner::roll.setTunings(setting.value);
      DEBUG(F("regInner_p changed to ")); DEBUGLN(setting.value);
    }
    break;
  case SettingId::regInner_yawP:
    if (setting.request) {
      DEBUGLN(F("Requested regInner_yawP"));
      setting.value = config::regulation::inner::yawP();
    } else {
      setting.success = config::regulation::inner::yawP.changeValue(setting.value);
      setting.value = config::regulation::inner::yawP();
      pid::inner::yaw.setTunings(setting.value);
      DEBUG(F("regInner_yawP changed to ")); DEBUGLN(setting.value);
    }
    break;
  case SettingId::regInner_outputLimit:
    if (setting.request) {
      DEBUGLN(F("Requested regInner_outputLimit"));
      setting.value = (float)config::regulation::inner::outputLimit();
    } else {
      setting.success = config::regulation::inner::outputLimit.changeValue(setting.value);
      setting.value = (float)config::regulation::inner::outputLimit();
      pid::inner::pitch.setOutputLimits(config::regulation::inner::outputLimit());
      pid::inner::roll.setOutputLimits(config::regulation::inner::outputLimit());
      pid::inner::yaw.setOutputLimits(config::regulation::inner::outputLimit());
      DEBUG(F("regInner_outputLimit changed to ")); DEBUGLN(setting.value);
    }
    break;
  case SettingId::regOuter_p:
    if (setting.request) {
      DEBUGLN(F("Requested regOuter_p"));
      setting.value = config::regulation::outer::P();
    } else {
      setting.success = config::regulation::outer::P.changeValue(setting.value);
      setting.value = config::regulation::outer::P();
      pid::outer::pitch.setTunings(setting.value, config::regulation::outer::I());
      pid::outer::roll.setTunings(setting.value, config::regulation::outer::I());
      DEBUG(F("regOuter_p changed to ")); DEBUGLN(setting.value);
    }
    break;
  case SettingId::regOuter_i:
    if (setting.request) {
      DEBUGLN(F("Requested regOuter_i"));
      setting.value = config::regulation::outer::I();
    } else {
      setting.success = config::regulation::outer::I.changeValue(setting.value);
      setting.value = config::regulation::outer::I();
      pid::outer::pitch.setTunings(config::regulation::outer::P(), setting.value);
      pid::outer::roll.setTunings(config::regulation::outer::P(), setting.value);
      DEBUG(F("regOuter_i changed to ")); DEBUGLN(setting.value);
    }
    break;
  case SettingId::regOuter_updateRate:
    if (setting.request) {
      DEBUGLN(F("Requested regOuter_updateRate"));
      setting.value = (float)config::regulation::outer::updateRate();
    } else {
      setting.success = config::regulation::outer::updateRate.changeValue(setting.value);
      setting.value = (float)config::regulation::outer::updateRate();
      pid::outer::pitch.setUpdateRate(config::regulation::outer::updateRate());
      pid::outer::roll.setUpdateRate(config::regulation::outer::updateRate());
      DEBUG(F("regOuter_updateRate changed to ")); DEBUGLN(setting.value);
    }
    break;
  case SettingId::regOuter_outputLimit:
    if (setting.request) {
      DEBUGLN(F("Requested regOuter_outputLimit"));
      setting.value = (float)config::regulation::outer::outputLimit();
    } else {
      setting.success = config::regulation::outer::outputLimit.changeValue(setting.value);
      setting.value = (float)config::regulation::outer::outputLimit();
      pid::inner::pitch.setOutputLimits(config::regulation::outer::outputLimit());
      pid::inner::roll.setOutputLimits(config::regulation::outer::outputLimit());
      pid::inner::yaw.setOutputLimits(config::regulation::outer::outputLimit());
      DEBUG(F("regOuter_outputLimit changed to ")); DEBUGLN(setting.value);
    }
    break;
  case SettingId::reg_minRegThrottle:
    if (setting.request) {
      DEBUGLN(F("Requested reg_minRegThrottle"));
      setting.value = (float)config::regulation::minimumRegulationThrottle();
    } else {
      setting.success = config::regulation::minimumRegulationThrottle.changeValue(setting.value);
      setting.value = (float)config::regulation::minimumRegulationThrottle();
      DEBUG(F("reg_minRegThrottle changed to ")); DEBUGLN(setting.value);
    }
    break;
  case SettingId::reg_maxBaseThrottle:
    if (setting.request) {
      DEBUGLN(F("Requested reg_maxBaseThrottle"));
      setting.value = (float)config::regulation::maximumBaseThrottle();
    } else {
      setting.success = config::regulation::maximumBaseThrottle.changeValue(setting.value);
      setting.value = (float)config::regulation::maximumBaseThrottle();
      DEBUG(F("reg_maxBaseThrottle changed to ")); DEBUGLN(setting.value);
    }
    break;

  case SettingId::indication_armsLevel:
    if (setting.request) {
      DEBUGLN(F("Requested indication_armsLevel"));
      setting.value = (float)config::indication::armsLevel();
    } else {
      setting.success = config::indication::armsLevel.changeValue(setting.value);
      setting.value = (float)config::indication::armsLevel();
      indication::arms(config::indication::armsLevel());
      DEBUG(F("indication_armsLevel changed to ")); DEBUGLN(setting.value);
    }
    break;
  case SettingId::indication_lamp:
    if (setting.request) {
      DEBUGLN(F("Requested indication_lamp"));
      setting.value = (float)config::indication::lamp();
    } else {
      setting.success = config::indication::lamp.changeValue(setting.value);
      setting.value = (float)config::indication::lamp();
      indication::lamp(config::indication::lamp());
      DEBUG(F("indication_lamp changed to ")); DEBUGLN(setting.value);
    }
    break;
  default:
    setting.success = false;
    DEBUG(F("Unrecognized setting id: ")); DEBUGLN((uint8_t)setting.id);
    break;
  } //switch (setting.id)
  comm::rf24.setResponse(&setting, sizeof(setting));
}
