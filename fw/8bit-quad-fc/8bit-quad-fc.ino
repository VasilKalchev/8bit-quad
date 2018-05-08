// License...

/* TODO:
// turn off if large acceleration and not moving

// Self tests with fake data
// set angular velocity z to positive value - check if motors react
// set control pitch to left - check if motors react
// check if Eaccel == 1g
// check if Eav == 0
// check if Az == 1g
// check if integrating correctly (pids and av yaw)
// check if loop time is constant and under certain value
*/

/* TOC

*/

// Nonvolatile preference increment based on data type

#define F_CPU 16000000


#include "MAD-FC-2a_config.hpp"

/* MAD-FC-2a_debug defines macros 'DEBUG_INITIALIZE(baud)', 'DEBUG(...)'
   and 'DEBUGLN(...)' based on the definition of 'DEBUGGING' in
   MAD-FC-2a_config.hpp. This allows turning on/off UART debugging
   messages from a single place. */
#include "MAD-FC-2a_debug.hpp"

/* MAD-FC-2a_board defines how the microcontroller is connected to board
   "2a" and provides functions for controlling the basic peripherals. */
#include "MAD-FC-2a_board.hpp"

#include <MAD-DataTypes.hpp>
#include <MAD-Utils.hpp>

/* UART_atmega328 library is an alternative to the Arduino's library for
   working with the UART. It is specifically for the ATmega328, it lacks a
   lot of functionality and is meant to be used for simple debugging
   messages while eliminating the requirement of using the Arduino's IDE. */
#include <UART_atmega328.hpp> // https://github.com/VaSe7u/UART_atmega328

/* ADC_atmega328 library is an alternative to the Arduino's functions for
   working with the ADC. It is specifically for the ATmega328 and provides
   functions for setting a trigger source and changing the ADC circuit's
   operating frequency. */
#include <ADC_atmega328.hpp> // https://github.com/VaSe7u/ADC_atmega328

#include <MPU925x_I2C.hpp> // https://github.com/VaSe7u/MPU925x_I2C
#include <AK8963_I2C.hpp> // https://github.com/VaSe7u/AK8963_I2C
#include <i2c_BMP280.h> // https://github.com/orgua/iLib

/* MahonyAHRS is a library implementing Mahony's orientation filter for
   fusing accelerometer, gyroscope and possibly magnetometer.
   This is a fork of https://github.com/PaulStoffregen/MahonyAHRS, the
   main difference is that angular velocity is passed to the filter in
   radians/s. */
#include <MahonyAHRS.h> // https://github.com/VaSe7u/MahonyAHRS

#include <PIDcontroller.hpp> // https://github.com/VaSe7u/PIDcontroller

#include <avr/wdt.h>


/* This function makes sure that the code that follows it, is executed at
   a constant period. It uses the cycle period, which is specified manually
   in MAD-FC-2a_config.hpp based on the longest time it takes for a cycle to
   complete with SYNCHRONIZE turned off and the last time (in microseconds)
   this part of the code was reached. The argument "lastMicroseconds" is
   passed as a reference and is automatically updated. */
void syncCycle(uint32_t& lastMicroseconds) {
# if SYNCHRONIZE
  while (us() - lastMicroseconds < config::cyclePeriod) {}
  lastMicroseconds = us();
# else
#   warning "SYNCHRONIZE is off!"
  lastMicroseconds = lastMicroseconds;
# endif
}


// Function prototypes.
void updateIndication();
float readBattery();
void updateTelemetryMessage();
void adjustControllers();
void updateTelemetryMessage();
bool commandHandler();
bool settingHandler();


int main() {
  wdt_disable();

  // MAD-FC-2a-board.hpp
  board::initializeTimers();
  /* Configures timer 0 and 1 for 490Hz Fast PWM for the motors
     and timer 2 for counting the microseconds since boot. */
  board::initIO(); // Configures the pins for input or output.

  board::indication::signal(false);
  board::indication::warning(true);
  board::indication::arms(0);
  board::indication::lamp(false);
  // Only red LED is lit while initializing.


  DEBUG_INITIALIZE(config::debug::baud);
  DEBUG("MAD-FC-2a\n=========\n\n");


  // Global objects.
  // ===============
  adc::initialize(adc::Reference::INTERNAL1V1); // link to API
  adc::setAutoTriggerSource(adc::AutoTriggerSource::FreeRunningMode);


  namespace imu {
  MPU925x_I2C mpu; // link to API
  bool init = true;
  mpu.initialize();
  init &= mpu.setAccelFullScaleRange(config::imu::mpu925x::accelerometer::range);
  init &= mpu.setGyroFullScaleRange(config::imu::mpu925x::gyroscope::range);
  init &= mpu.setAccelDLPF(config::imu::mpu925x::accelerometer::dlpf);
  init &= mpu.setGyroDLPF(config::imu::mpu925x::gyroscope::dlpf);
  mpu.setXAccelOffset(board::imu::mpu925x::offset::accelerometer::x());
  mpu.setYAccelOffset(board::imu::mpu925x::offset::accelerometer::y());
  mpu.setZAccelOffset(board::imu::mpu925x::offset::accelerometer::z());
  mpu.setXGyroOffset (board::imu::mpu925x::offset::gyroscope::x());
  mpu.setYGyroOffset (board::imu::mpu925x::offset::gyroscope::y());
  mpu.setZGyroOffset (board::imu::mpu925x::offset::gyroscope::z());
  init &= mpu.testConnection();

  // AK8963_I2C compass;
  } //namespace imu


  namespace altimeter {
  BMP280 bmp;
  bool init = true;
  bmp.initialize();
  bmp.setPressureOversampleRatio(config::altimeter::bmp280::pressureOversampleRatio);
  bmp.setTemperatureOversampleRatio(config::altimeter::bmp280::temperatureOversampleRatio);
  bmp.setFilterRatio(config::altimeter::bmp280::filterRatio);
  bmp.setStandby(config::altimeter::bmp280::standbyTime);
  bmp.setEnabled(true);
  } //namespace altimeter


  namespace fusion {
  Mahony mahony(config::cyclePeriod);
  mahony.setP(config::fusion::mahony::p);
  mahony.setI(config::fusion::mahony::i);
  } //namespace fusion


  namespace controller {
  bool init = true;
  namespace inner {

  const PIDconfig pitchAndRollConfig;
  pitchAndRollConfig.state = true;
  pitchAndRollConfig.p = config::controller::inner::p();
  pitchAndRollConfig.updatePeriod = config::controller::inner::updatePeriod;
  pitchAndRollConfig.minimalOutput = 0 - config::controller::inner::outputRange;
  pitchAndRollConfig.maximalOutput = config::controller::inner::outputRange;

  const PIDconfig yawConfig;
  yawConfig.state = true;
  yawConfig.p = config::controller::inner::yaw::p();
  yawConfig.i = config::controller::inner::yaw::i();
  yawConfig.updatePeriod = config::controller::inner::updatePeriod;
  yawConfig.minimalOutput = 0 - config::controller::inner::outputRange;
  yawConfig.maximalOutput = config::controller::inner::outputRange;
  yawConfig.maximalNegativeIntegralSum = 0 - config::controller::inner::yaw::maximalIntegralSum;
  yawConfig.maximalPositiveIntegralSum = config::controller::inner::yaw::maximalIntegralSum;

  PIDcontroller pitch;
  init &= pitch.config(pitchAndRollConfig);
  PIDcontroller roll;
  init &= roll.config(pitchAndRollConfig);
  PIDcontroller yaw;
  init &= yaw.config(yawConfig);
  } //namespace inner

  namespace outer {

  const PIDconfig pitchAndRollConfig;
  pitchAndRollConfig.state = true;
  pitchAndRollConfig.p = config::controller::outer::p();
  pitchAndRollConfig.i = config::controller::outer::i();
  pitchAndRollConfig.updatePeriod = config::controller::outer::updatePeriod;
  pitchAndRollConfig.minimalOutput = 0 - config::controller::outer::outputRange;
  pitchAndRollConfig.maximalOutput = config::controller::outer::outputRange;
  pitchAndRollConfig.maximalNegativeIntegralSum = 0 - config::controller::outer::maximalIntegralSum;
  pitchAndRollConfig.maximalPositiveIntegralSum = config::controller::outer::maximalIntegralSum;

  PIDcontroller pitch;
  init &= pitch.config(pitchAndRollConfig);
  PIDcontroller roll;
  init &= roll.config(pitchAndRollConfig);
  } //namespace outer
  } //namespace controller


  namespace com {
  RF24 rf24(7, 8); // Arduino pins 7 and 8.
  bool init = true;
  rf24.begin();
  rf24.setPALevel((rf24_pa_dbm_e)config::communication::nrf24l01p::paLevel);
  rf24.setDataRate((rf24_datarate_e)config::communication::nrf24l01p::dataRate);
  rf24.setRetries(config::communication::nrf24l01p::retriesCount,
                  config::communication::nrf24l01p::retriesDelay);
  rf24.setCRCLength((rf24_crclength_e)config::communication::nrf24l01p::crcLength);
  rf24.enableAckPayload();
  rf24.enableDynamicPayloads();
  rf24.enableDynamicAck();
  rf24.setAddressWidth(config::communication::nrf24l01p::addressWidth);
  uint8_t writingPipe[] = {config::communication::remotePipe,
                           config::communication::pipe, config::communication::pipe
                          };
  uint8_t readingPipe[] = {config::communication::dronePipe,
                           config::communication::pipe, config::communication::pipe
                          };
  rf24.openWritingPipe(writingPipe);
  rf24.openReadingPipe(1, readingPipe);
  rf24.startListening();
  init = rf24.testConnection();
  } //namespace com

  // ---------------
  // ===============


  if (!imu::init || !com::init || !controller::init || !altimeter::init) {
    DEBUG("Failed initializing:");
    if (!imu::init) DEBUG(" IMU");
    if (!com::init) DEBUG(" communication");
    if (!controller::init) DEBUG(" controller");
    if (!altimeter::init) DEBUG(" altimeter");
    DEBUGLN("!");
    while (1) {
      board::indication::warningToggle(); _delay_ms(100);
    }
  }
  board::indication::signal(true);
  board::indication::warning(false);
  board::indication::arms(0);
  board::indication::lamp(config::indication::lampState());

  // Read battery voltage
  // if Vb < 5 print settings and records and enable recording records

  wdt_reset();
  wdt_enable(WDTO_2S);


  for (;;) {
    wdt_reset();

    // Cycles.
    // -------
    static int8_t thisCycle = 0;
    if (thisCycle > config::cycles) thisCycle = 0; // 3 cycles: 0, 1, 2
    else ++thisCycle;
    // -------

    static uint32_t usThisCycle = us();
#   if DEBUG_CYCLE_PERIOD
    uart::print("CP: "); uart::println(us() - usThisCycle);
#   endif
    usThisCycle = us(); // Calculate us() once for non-critical usage.


    // Read the IMU.
    // =============
    static mad::imu::AccelerationRaw accelerationRaw;
    static mad::imu::AngularVelocityRaw angularVelocityRaw;
    /* The accelerations and angular velocities are persistent
       in case the read fails. */
    imu.getAccelAndGyroRaw(&accelerationRaw.x, &accelerationRaw.y, &accelerationRaw.z,
                           &angularVelocityRaw.x, &angularVelocityRaw.y, &angularVelocityRaw.z);

    // Convert gyroscope's raw output to radians/s.
    // AV[deg/s] = Raw output / LSB/(deg/s)
    // AV[rad/s] = Raw output / LSB/(deg/s) * pi/180
    static mad::imu::AngularVelocity angularVelocityRadS;
    angularVelocityRadS.x = imu.gyroRawToRadS(angularVelocityRaw.x);
    angularVelocityRadS.y = imu.gyroRawToRadS(angularVelocityRaw.y);
    angularVelocityRadS.z = imu.gyroRawToRadS(angularVelocityRaw.z);

    // static mad::imu::MagneticFieldRaw magneticFieldRaw;
    // compass.getMagneticFieldRaw(&magneticFieldRaw.x, &magneticFieldRaw.y, &magneticFieldRaw.z);
    /* No need to convert the accelerations and magnetic fields because
       they are normalised by the Mahony filter. */

    // float pressure = 0.0f;
    // float temperature = 0.0f;
    // altimeter::bmp.read(pressure, temperature);
    // static float takeOffPressure = pressure;

#   if DEBUG_ANGULAR_VELOCITY
    uart::print(imu.gyroRawToDegS(angularVelocityRaw.x)); uart::print(", ");
    uart::print(imu.gyroRawToDegS(angularVelocityRaw.y)); uart::print(", ");
    uart::print(imu.gyroRawToDegS(angularVelocityRaw.z)); uart::print("\n");
#   endif
#   if DEBUG_ACCELERATION
    uart::print(accelerationRaw.x); uart::print(", ");
    uart::print(accelerationRaw.y); uart::print(", ");
    uart::print(accelerationRaw.z); uart::print("\n");
#   endif
#   if DEBUG_MAGNETIC_FIELD
    uart::print(magneticFieldRaw.x); uart::print(", ");
    uart::print(magneticFieldRaw.y); uart::print(", ");
    uart::print(magneticFieldRaw.z); uart::print("\n");
#   endif
#   if DEBUG_PRESSURE
    uart::print(pressure); uart::print("\n");
#   endif
    // =============


    // Calculate the attitude.
    // ======================= (sync block)
    mad::imu::Attitude attitude;

    static uint32_t mahonyLUS = usThisCycle;
#   if DEBUG_SYNC_PERIOD
    uart::print("SP: "); uart::println(us() - mahonyLUS);
#   endif
    syncCycle(&mahonyLUS);
    /* The next line of code is executed at an exact period, if
       config::cyclePeriod in MAD-FC-2a_config.hpp is set correctly. */
    mahony.update(&attitude.yaw, &attitude.pitch, &attitude.roll,
                  accelerationRaw.x, accelerationRaw.y, accelerationRaw.z,
                  angularVelocityRadS.x, angularVelocityRadS.y, angularVelocityRadS.z);
    attitude.yaw = mad::utils::radiansToDegrees(attitude.yaw);
    attitude.pitch = mad::utils::radiansToDegrees(attitude.pitch);
    attitude.roll = mad::utils::radiansToDegrees(attitude.roll);


    // Calculate the angular velocities.
    // ---------------------------------
    mad::imu::AngularVelocity angularVelocityDegS;
    static mad::imu::Attitude attitudeLast;

    static uint32_t dAvLUS = usThisCycle; // delta angular velocity
    uint32_t usV = us(); // microseconds velocity
    float dtV = (usV - dAvLUS) / 1000000.0f; // dt velocity
    // stopwatch the time to the actual instruction and add it to dAvLUS
    angularVelocityDegS.z = (attitude.yaw - attitudeLast.yaw) / (dtV + 0.000016f);
    /* The difference in angle with respect to time is angular velocity. The small
       number added to dtV is the time passed since dtV was calculated. */
    attitudeLast.yaw = attitude.yaw;
    if (config::imu::deriveFromFused) {
      /* The angular velocity of x and y can be taken directly from the gyroscope's
         output or it can be calculated from the fused angle. The later should be
         less noisy. */
      angularVelocityDegS.x = (attitude.pitch - attitudeLast.pitch) / (dtV + 0.000032f);
      // stopwatch the time to this instruction and add it to dtV in seconds
      angularVelocityDegS.y = (attitude.roll - attitudeLast.roll) / (dtV + 0.000036f);
      // stopwatch the time to this instruction and add it to dtV in seconds
      attitudeLast.pitch = attitude.pitch;
      attitudeLast.roll = attitude.roll;
    } else {
      angularVelocityDegS.x = imu.gyroRawToDegS(angularVelocityRaw.x);
      angularVelocityDegS.y = imu.gyroRawToDegS(angularVelocityRaw.y);
      /* Yaw's angular velocity is always derived from the fused yaw angle
         since it is definetly more accurate. */
    }
    dAvLUS = usV;
    // ---------------------------------
    // =======================


    // Calculate the controllers.
    // ==========================
    static mad::Control control; // This object holds mainly the user's input.
    static float throttle = control.throttle;
    static float yawDifferential = 0.0f;
    static float pitchDifferential = 0.0f;
    static float rollDifferential = 0.0f;
    /* The 'differential' variables will be calculated from the controllers and
       then will be added to the motors to create the desired "pull" difference. */

    if (control.holdAltitude) {
      // set control.altitude from throttle, when throttle centers set current altitude
      // control.verticalVelocity = controller::altitude.calculate(control.altitude,
      // altitude, control.verticalVelocity);
      // throttle = controller::verticalVelocity.calculate(control.verticalVelocity,
      // verticalVelocity, throttle);
    } else {
      throttle = control.throttle;
    }

    if (config::controller::type == mad::ControllerType::PID) {
      // PID variant.
      // ------------

      // ------------ ~PID
    } else if (config::controller::type == mad::ControllerType::PI_P) {
      // PI-P variant.
      // -------------
      // Outer (every third cycle).
      /* The outer controller loop compares the desired angle to the actual
         one and calculates an appropriate output. In this case the output
         is desided to be velocity... */
      if (thisCycle == config::controller::outer::cycle) {
        static uint32_t outerLUS = usThisCycle;
        while (us() - outerLUS < config::cyclePeriod * (config::cycles + 1));
        outerLUS = us();
        control.rollVelocity = controller::outer::pitch.calculate(control.pitchAngle,
                               attitude.pitch, control.rollVelocity);
        control.pitchVelocity = controller::outer::roll.calculate(control.rollAngle,
                                attitude.roll, control.pitchVelocity);
      }
      // Inner (every cycle).
      /* The inner controller loop uses the output from the outer loop as the
         desired angular velocity and compares it with the actual angular
         velocity. The output is desided to be difference in motor "pull"...  */
      static uint32_t innerLUS = usThisCycle;
      syncCycle(&innerLUS);
      yawDifferential = controller::inner::yaw.calculate(control.yawVelocity,
                        angularVelocityDegS.z, yawDifferential);
      pitchDifferential = controller::inner::pitch.calculate(control.rollVelocity,
                          angularVelocityDegS.z, pitchDifferential);
      rollDifferential = controller::inner::roll.calculate(control.pitchVelocity,
                         angularVelocityDegS.z, rollDifferential);
      // ------------- ~PI-P
    } else {
      config::controller::type == mad::ControllerType::PID; // default to PID
    }
    // ==========================


    // Mix the motors.
    // ===============
    /* The output from the inner controller loop is a number representing
       the difference in motor "pull" required to achieve the user's input.
       This number can be positive, negative or zero and it is added or
       substracted to the appropriate motors (e.g. the pitch output is
       added/substracted from the front motors and substracted/added to the
       back ones). The sign depends on the IMU's output and the controller's
       direction (the input from the remote is corrected based on the IMU's
       output). To control yaw there needs to be a difference in gyroscopic
       effect (e.g. to rotate the drone CCW the motors rotating CW have to
       be rotating faster than the motors rotating CCW). In this case the
       top-left motor is rotating CCW, top-right - CW, bottom-right - CCW and
       bottom-left - CW. Since the mechanics of controlling yaw is different
       than the one controlling pitch and roll (difference in motor "pull"
       generates less angular velocity through gyroscopic effect compared to
       aerodynamic one), there needs to be a different controller with it's
       own configuration for controlling yaw. */
    // !!! move some to beginning
    static uint32_t motorMixLUS = 0;
    syncCycle(&motorMixLUS);
    uint8_t tl = 0, tr = 0, bl = 0, br = 0;
    if (throttle > config::minimumRegulationThrottle) {
      /* The controllers' outputs are ignored at very low throttle,
         otherwise the proppelers will rotate when the drone is not
         level or moving (e.g. being carried). */
      if (throttle > config::maximumBaseThrottle) {
        throttle = config::maximumBaseThrottle;
        /* The maximum throttle settable by the user is lower than actual
           maximum one. This is to avoid losing half of the controllers'
           outputs when at full throttle. */
      }
      tl = (uint8_t)mad::util::clamp(round(throttle
                                           + pitchDifferential
                                           + rollDifferential
                                           - yawDifferential), 0, 127);
      tr = (uint8_t)mad::util::clamp(round(throttle
                                           + pitchDifferential
                                           - rollDifferential
                                           + yawDifferential), 0, 127);
      bl = (uint8_t)mad::util::clamp(round(throttle
                                           - pitchDifferential
                                           + rollDifferential
                                           + yawDifferential), 0, 127);
      br = (uint8_t)mad::util::clamp(round(throttle
                                           - pitchDifferential
                                           - rollDifferential
                                           - yawDifferential), 0, 127);
      /* The result from mixing the controllers' outputs and the base throttle
         is rounded and then clamped to the settable range of the motors. */
    } else {
      uint8_t t = (uint8_t)mad::util::clamp(round(throttle), 0, 127);
      tl = t; tr = t;
      bl = t; br = t;
      /* If the throttle is below the minimum regulation margin it is directly
         applied to the motors. */
      controller::inner::yaw.unwindIntegralSum(angularVelocityDegS.z, yawDifferential);
      controller::outer::pitch.unwindIntegralSum(attitude.pitch, control.rollVelocity);
      controller::outer::roll.unwindIntegralSum(attitude.roll, control.pitchVelocity);
      /* The integral sum is reset while at very low throttle (usually the drone
         will be on the ground) to avoid accumulating large integral error for
         no reason. */
    }
    OCR0B = 127 + tl; OCR1B = 127 + tr;
    OCR0A = 127 + bl; OCR1A = 127 + br;
    /* The motors' drivers are controlled with PPM signals. Half PWM is off,
       (full PWM - 1) in full on. */
    // ===============


    // Low priority (indication, additional sensors...).
    // =================================================
    float batteryVoltage = 0.0f;
    if (thisCycle == config::lowPriorityCycle) {
      static int8_t subCycle = 0;

      if (subCycle == 0) {
        static uint32_t indicationLUS = 0;
        if (usThisCycle - indicationLUS > config::indication::period) {
          updateIndication();
        }
      } else if (subCycle == 1) {
        // uint32_t m = us(); // stopwatch the maximum allowed time
        batteryVoltage = readBattery();
        // DEBUG(us() - m);
      } else {
        // Do scheduled work (with timeouts).
        // zero out angular velocity if on the ground
        // update statistics in eeprom
        // - battery, horizontal speed, motor power
        // - record altitude, record horizontal speed
        // - time flying
        // - motor energy

        subCycle = -1;
      }
      ++subCycle;
    } // if correct cycle
    // =================================================


    // Check for new message (every 3rd cycle).
    // ========================================
    if (thisCycle == config::communication::cycle) {
      static uint32_t commandLUS = usThisCycle;
      static bool justReceived = false;
      static bool readyForTelemetry = false;

      if (justReceived) {
        adjustControllers();
        justReceived = false;
        readyForTelemetry = true;
      } else if (usThisCycle - commandLUS > config::communication::timeout) {
        control.flyMode = mad::FlyMode::noConnection;
      } else if (readyForTelemetry) {
        updateTelemetryMessage();
        readyForTelemetry = false;
      }

      while (com::rf24.available()) {
        const uint8_t size = com::rf24.getDynamicPayloadSize();
        if (size != 0) {
          uint8_t messageRaw[size];
          com::rf24.read(&messageRaw, size);
          if (messageRaw[0] == mad::communication::MessageType::command) {
            if (communication::commandHandler(&messageRaw, size)) {
              commandLUS = usThisCycle;
              justReceived = true;
            }
          } else if (messageRaw[0] == mad::communication::MessageType::setting) {
            communication::settingHandler(&messageRaw);
          } else {
            //error
            com::rf24.read(0, 0);
          }
        } // if size is valid
      } // while there is a message
    } // if correct cycle
    // ========================================

  } // main cycle
  return 0;
}


void updateIndication() {
  static uint8_t levelSetpoint = config::indication::armsLevel();
  const uint8_t levelCurrent = board::indication::armsLevel();
  uint8_t levelNew = 0;
  if (levelCurrent < levelSetpoint) {
    levelNew = levelCurrent + 1;
  } else if (levelCurrent > levelSetpoint) {
    levelNew = levelCurrent - 1;
  }
  /* The above code achieves gradual change in brightness for the arms'
     LEDs. The level setpoint is set later based on the state of the drone. */
  board::indication::arms(pgm_read_byte(&exp[mad::utils::clamp(levelNew, 0, 36)]));
  /* The actual brighness is an exponential function stored as array in
     PROGMEM. For more information: https://github.com/VaSe7u/ExponentMap */

  static uint32_t indicationLUS = usThisCycle;
  if (usThisCycle - indicationLUS > config::indication::period) {
    indicationLUS = usThisCycle;
    /* Indication pattern (from highest to lowest priority):
         No connection - arms blink and LEDs blink;
         Landing - arms blink;
         Critical battery - arms blink (full on - almost full on) and red LED blink;
         Low battery - green and red LED blink;
         Altitude hold - arms fade (full on - almost full on);
         Not in angle fly mode - arms fade (full on - mid);
    */

    // Arms LEDs
    static bool armsStep = 0;
    armsStep = !armsStep;
    if (control.flyMode == mad::FlyMode::noConnection || control.flyMode == mad::FlyMode::land) {
      board::indication::armsToggle();
    } else if (batteryVoltage < config::battery::criticalMargin) {
      (armsStep) ? levelSetpoint = 36 : levelSetpoint = 27;
      board::indication::arms(pgm_read_byte(&exp[mad::utils::clamp(levelSetpoint, 0, 36)]));
    } else if (control.holdAltitude) {
      (armsStep) ? levelSetpoint = 36 : levelSetpoint = 27;
    } else if (control.flyMode != mad::FlyMode::angle) {
      (armsStep) ? levelSetpoint = 36 : levelSetpoint = 18;
    } else {
      levelSetpoint = config::indication::armsLevel();
    }

    // Board LEDs
    if (control.flyMode == mad::FlyMode::noConnection) {
      board::indication::signal(!board::indication::armsState());
      board::indication::warning(!board::indication::armsState());
    } else if (batteryVoltage < config::battery::criticalMargin) {
      board::indication::signal(false);
      board::indication::warningToggle();
    } else if (batteryVoltage < config::battery::lowMargin) {
      board::indication::signalToggle();
      board::indication::warning(board::indication::signalState());
    } else {
      board::indication::signalToggle();
      board::indication::warning(false);
    }
  }
}

float readBattery() {
  float batteryVoltage = static_cast<float>(adc::read(pin::input::batteryVoltage)
                         * board::battery::readingMultiplier);
  /* The battery's ADC reading is converted to voltage with a previously
     calculated multiplier defined in MAD-FC-2a_board.hpp. */
  static float batteryVoltageLast = batteryVoltage;
  mad::util::lowPassFilter(&batteryVoltage, &batteryVoltageLast,
                           config::battery::lpfAlpha);
  return batteryVoltage;
}

void adjustControllers() {
  if (config::controller::type == ControllerType::PI_P) {
    // PI-P
    if (control.flyMode == mad::FlyMode::angle) {
      // Control by angle.
      controller::outer::pitch.on();
      controller::outer::roll.on();
      control.pitchAngle = control.pitch;
      control.rollAngle = control.roll;
    } else if (control.flyMode == mad::FlyMode::acro) {
      // Control by angular velocity.
      controller::outer::pitch.off();
      controller::outer::roll.off();
      control.pitchVelocity = control.pitch;
      control.rollVelocity = control.roll;
    } else if (control.flyMode == mad::FlyMode::horizon) {
      // If tilt is below certain margin 'angle', else 'acro'.
      if (abs(control.pitch) < config::horizonMargin) { //angle pitch
        controller::outer::pitch.on();
        control.pitchAngle = control.pitch;
      } else { //acro pitch
        controller::outer::pitch.off();
        control.pitchVelocity = control.pitch;
      }
      if (abs(control.roll) < config::horizonMargin) { //angle roll
        controller::outer::roll.on();
        control.rollAngle = control.roll;
      } else { //acro roll
        controller::outer::roll.off();
        control.rollVelocity = control.roll;
      }
    } else if (control.flyMode == mad::FlyMode::land) {
      // stab and turn on altitude controller
      // continuaously decrease control.altitude
    } else if (control.flyMode == mad::FlyMode::direct) {
      // turn off everything
    } else {
      control.flyMode = mad::FlyMode::angle;
      // Control by angle.
      controller::outer::pitch.on();
      controller::outer::roll.on();
      control.pitchAngle = control.pitch;
      control.rollAngle = control.roll;
    }
  } else if (config::controller::type == ControllerType::PID) {
    // PID

  } else {
    config::controller::type = ControllerType::PID; // default to PID controller type
  } // controller type
}


void updateTelemetryMessage() {
  /* The telemetry message will be send immediately when a command
     message is received. It needs to be preloaded. */
  switch (config::communication::telemetryType()) {
  case mad::communication::MessageType::telemetryNormal;
    mad::communication::TelemetryNormal telemetry;
    telemetry.batteryVoltage_m = (uint16_t)round(batteryVoltage * 1000.0f);
    telemetry.altitude_c = (int16_t)round(altitude * 100.0f); // max: 320m
    com::rf24.writeAckPayload(1, telemetry, sizeof(telemetry)); // pipe 1
    break;
  default:
    config::communication::telemetryType = mad::communication::MessageType::telemetryNormal;
    break;
  }
}

bool commandHandler(const uint8_t& messageRaw, const uint8_t size) {
  mad::communication::Command command;
  /* Most of the values in struct Command are in centi (1e2) (centi-degrees,
     centi-throttle), to get the 'actual' values divide by 100. */
  memcpy(&command, &messageRaw, size);

  static int8_t remoteId = command.remoteId;
  if (command.remoteId != remoteId
      || command.throttle_c < 0 || command.throttle_c > 12700
      || command.pitch_c < -30000 || command.pitch_c > 30000
      || command.roll_c < -30000 || command.roll_c > 30000
      || command.yaw_c < -30000 || command.yaw_c > 30000
      || (int8_t)command.flyMode < (int8_t)mad::FlyMode::first
      || (int8_t)command.flyMode > (int8_t)mad::FlyMode::last) {
    return false;
  } else {
    control.throttle = (float)command.throttle_c / 100.0f;
    control.pitch = (float)command.pitch_c / 100.0f;
    control.roll = (float)command.roll_c / 100.0f;
    control.yawVelocity = (float)command.yaw_c / 100.0f;
    control.flyMode = command.flyMode;
    control.holdAltitude = command.holdAltitude;
    return true;
  }
}

bool settingHandler(const uint8_t& messageRaw, const uint8_t size) {
  // Indication feedback
  board::indication::signal(true);
  board::indication::warning(true);

  mad::communication::Setting setting;
  memcpy(&setting, &messageRaw, size);
  setting.success = false;

  switch (setting.id) {
  case mad::SettingId::innerP:
    if (!setting.request) {
      setting.success = (config::controller::inner::p = setting.value);
    }
    setting.value = config::controller::inner::p();
    controller::inner::pitch.setP(config::controller::inner::p());
    controller::inner::roll.setP(config::controller::inner::p());
    break;
  case mad::SettingId::innerYawP:
    if (!setting.request) {
      setting.success = (config::controller::inner::yaw::p = setting.value);
    }
    setting.value = config::controller::inner::yaw::p();
    controller::inner::yaw.setP(config::controller::inner::yaw::p());
    break;
  case mad::SettingId::innerYawI:
    if (!setting.request) {
      setting.success = (config::controller::inner::yaw::i = setting.value);
    }
    setting.value = config::controller::inner::yaw::i();
    controller::inner::yaw.setI(config::controller::inner::yaw::i());
    break;
  case mad::SettingId::outerP:
    if (!setting.request) {
      setting.success = (config::controller::outer::p = setting.value);
    }
    setting.value = config::controller::outer::p();
    controller::outer::pitch.setP(config::controller::outer::p());
    controller::outer::roll.setP(config::controller::outer::p());
    break;
  case mad::SettingId::outerI:
    if (!setting.request) {
      setting.success = (config::controller::outer::i = setting.value);
    }
    setting.value = config::controller::outer::i();
    controller::outer::pitch.setI(config::controller::outer::i());
    controller::outer::roll.setI(config::controller::outer::i());
    break;
  case mad::SettingId::armsLevel:
    if (!setting.request) {
      setting.success = (config::indication::armsLevel = setting.value);
    }
    setting.value = config::indication::armsLevel();
    break;
  case mad::SettingId::lampState:
    if (!setting.request) {
      setting.success = (config::indication::lampState = setting.value);
    }
    setting.value = config::indication::lampState();
    board::indication::lamp(config::indication::lampState());
    break;
  case mad::SettingId::telemetryType:
    if (!setting.request) {
      setting.success = (config::communication::telemetryType = setting.value);
    }
    setting.value = config::communication::telemetryType();
    break;
  case mad::SettingId::calibrateImu:
// if throttle is zero
// lock drone and start calibration
// save old calibration, if throttle goes high set it and abort calibration
    break;
  default:

    break;
  }

  com.stopListening();
  bool writeSuccess = com.write(&setting, sizeof(setting), 1);
  com.startListening();

  return (setting.success && writeSuccess);
}
