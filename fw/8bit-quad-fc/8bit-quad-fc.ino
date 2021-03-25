/* TODO:
 * DynamicCfg
 * Default config
 * EEPROM map
 * Save records when radio get turned off
 * Detect free-fall when no comms.
 * Extend ESC ranges
 * Make throttle more sensitive in the middle
 * Detect crashes by measuring angle in angle mode, no communication etc...
 * Detect idle state.
 * Check connection by measuring send time on different channels.
 * Apply SW trim while flying.
 * Send structures signatures in a generic packet when initializing comms.
 * Stream SD card contents to remote or relay connected to PC.
 * Test SD card speed and use it only if it is fast enought.
 */

/* Standard sensor axis frame:
 * pitch forward: acclrm.y-, gyro.x-
 *       backward: acclrm.y+, gyro.x+
 * roll right: acclrm.x-, gyro.y+
 *      right: acclrm.x+, gyro.y-
 * yaw ccw: gyro.z+
 *     cw: gyro.z-
 */

#include "config.hpp"

#include "src/lib/hw/m328/ADC/ADC.hpp"
#include "src/lib/hw/m328/UART/src/UART.hpp"
#include "src/lib/hw/BMP280/i2c_BMP280.h"

#include "src/lib/sw/MahonyAHRS/src/MahonyAHRS.h"
#include "src/lib/sw/PID/PID.hpp"
#include "src/lib/sw/util/util.hpp"

#include "src/debug.hpp"


#include "mad-fc-2_board.hpp"
#include "eeprom.hpp"


// Adapters
#include "nRF24L01p.hpp"

#if defined(IMU_MPU6050)
#include "MPU6050.hpp"
#elif defined(IMU_MPU925x)
#include "MPU9255.hpp"
#endif

using namespace m328;


enum class FlightMode : uint8_t {
  Acro, Stab,
};

enum class ElevHold : bool {
  Enable = true, Disable = false,
};

enum class Sky : bool {
  Enable = true, Disable = false,
}

struct State_t {
  FlightMode flight_mode;
  ElevHold elev_hold;
  Sky sky;
  uint16_t batt_v;
  int32_t take_off_elev_mm;

};

/* Global instances
 * 
 */
// Comm
namespace comm {
nRF24L01p_RF24 rf24(Role::drone,
  pin::communication::ino::ce, pin::communication::ino::csn);
} //namespace comm

Command command;
int16_t throttle = 0;

// IMU

// Att fusion
Mahony mahony;

// Elev
BMP280 bmp;
int32_t takeOffAltitude_mm = 0;
float takeOffPressure = 0.0f;

float relativeAltitude = 0.0f; // needs changes in telemetry packet
float recordRelativeAltitude = 0.0f;
const float seaLevelPressure = 101325.0f;

int16_t vertSpeed_sp_mmps = 0; // ctlr::vertSpeed sp
int16_t vertSpeed_mmps = 0; // ctlr::vertSpeed Input
float vertSpeed_input_mmps = 0.0f; // ctlr::vertSpeed Input
int16_t throttle_output = 0; // ctlr::vertSpeed Output

bool altHold = false;
uint32_t altHold_enTime = 0;
uint32_t const altHold_timeOut = 1000000;


// Ctlr
namespace ctlr {

namespace att {

namespace rate {
namespace sp {
int16_t pitch = 0;
int16_t roll = 0;
} // namespace sp

namespace output {
int16_t pitch = 0;
int16_t roll = 0;
int16_t yaw = 0;
} // namespace output

Pcontroller pitch(&sp::pitch, &angularVelocity.x, &output::pitch);
Pcontroller roll(&sp::roll, &angularVelocity.y, &output::roll);
PID yaw(&command.yaw, &avYaw, &output::yaw);
} // namespace rate

namespace angle {
PID pitch(&command.pitch, &attitude.pitch, &rate::sp::pitch);
PID roll(&command.roll, &attitude.roll, &rate::sp::roll);
} // namespace angle

} // namespace att


namespace elev {

// namespace pos {
// namespace sp {

// } // namespace sp
// namespace pv {

// } // namespace pv
// namespace op {

// } // namespace op
// } // namespace pos
// Pcontroller pos();


namespace rate {
namespace sp {

} // namespace sp
namespace pv {

} // namespace pv
namespace op {

} // namespace op
} // namespace rate
PID rate(&vertSpeed_sp_mmps, &vertSpeed_input_mmps, &throttle_output);


// namespace accel {
// namespace sp {

// } // namespace sp
// namespace pv {

// } // namespace pv
// namespace op {

// } // namespace op
// } // namespace accel
// Pcontroller accel();

} // namespace elev


namespace nav {

} // namespace nav

} //namespace ctlr





struct BatteryThrottleBuffer {
  uint16_t battery;
  uint8_t throttle;
};

bool battery_throttle_update = false;
BatteryThrottleBuffer battery_throttle_buffer;





namespace imu {
#if defined(IMU_MPU6050)
MPU6050_i2cDev mpu(config::imu::i2cAddress);
#elif defined(IMU_MPU925x)
MPU9255 mpu(config::imu::i2cAddress);
#endif
} //namespace imu

// Datatypes declared in "IMU.hpp"
AngularVelocity angularVelocity;
Acceleration acceleration;
Attitude attitude;
float avPitch = 0.0f;
float avRoll = 0.0f;
float avYaw = 0.0f;




namespace status {
Status communication = Status::normal;
Status battery = Status::normal;
} //namespace status

float batteryVoltage = 0.0;


void handleCommand();
void handleSetting(Setting & setting);

#define SYNC_CYCLE(LUS) while (micros() - LUS < config::cycleTime);


void setup() {
  bool success = true;

  replaceTimer0WithTimer2();

  initIO();
  indication::signal(OFF);
  indication::warning(ON);
  indication::arms(OFF);

  Wire.begin();
  Wire.setClock(800000); // 400000

  DBG_PIN_INIT();

  INIT_UART(config::debug::baud);
  uart::initialize(config::debug::baud);
  DEBUGLN("MAD 0.\n");

  config::init();
  #if DEBUG_SETTINGS
  PRINT("Preferences:\n");
  PRINT("\tPi: "); PRINT(config::ctlr::att::rate::P()); PRINT("\n");
  PRINT("\tPi_yaw: "); PRINT(config::ctlr::att::rate::yawP()); PRINT("\n");
  PRINT("\tPo: "); PRINT(config::ctlr::att::angle::P()); PRINT("\n");
  PRINT("\tIo: "); PRINT(config::ctlr::att::angle::I()); PRINT("\n");
  PRINT("\tTelemetry type: "); PRINT(config::communication::telemetry::type()); PRINT("\n");
  PRINT("\tArms level (0-12): "); PRINT(config::indication::armsLevel()); PRINT("\n");
  PRINT("\tLamp: "); PRINT(config::indication::lamp()); PRINT("\n");
  PRINT("\t-----\n");
  #endif

  #if DEBUG_BLACK_BOX_HOVER_VOLTAGE
  PRINT("Hover voltage\n");
  for (uint16_t i = EE_FIRST_FREE_ADDRESS; i < 1000; i += 3) {

    BatteryThrottleBuffer buff;
    while (eeprom_is_ready() == false) {}
    eeprom_read_block(&buff, i, sizeof(buff));

    PRINT(buff.battery); PRINT("\t"); PRINT(buff.throttle);
    PRINT("\n");
  }
  while (eeprom_is_ready() == false) {}
  uint32_t m = eeprom_read_dword(1005);
  PRINT("Out of memory at "); PRINT(m); PRINT("millis.\n");
  #endif

  #if DEBUG_EEPROM
  PRINT("EEPROM\n");
  for (uint16_t i = 0; i < 1023; ++i) {
    uint8_t ee_val = 0;
    eeprom::read(i, ee_val);
    PRINT("["); PRINT(i); PRINT("]: "); PRINT(ee_val); PRINT(" ");
    if (i % 10 == 0) PRINT("\n");
  }
  PRINT("\n");
  #endif

  initADC();

  // IMU initialization
  PRINT("IMU");
  success &= imu::mpu.initialize();
  PRINT(" done. T = "); PRINT(imu::mpu.getTemperature()); PRINT("degC.");
  // ~IMU initialization -----


  // Communication initialization
  PRINT("Communication");
  comm::rf24.initialize();
  comm::rf24.setPALevel(RF24_PA_MAX);
  comm::rf24.setDataRate(RF24_2MBPS);
  comm::rf24.setRetries(config::communication::retryDelay, config::communication::retryCount);
  comm::rf24.setCRCLength(config::communication::crcLength);
  success &= comm::rf24.isChipConnected();
  if (comm::rf24.isChipConnected()) { PRINT(" done.\n"); }
  // ~Communication initialization -----

  mahony.begin(322);

  // Regulation initialization
  // DEBUG("Regulation");
  success &= ctlr::att::rate::pitch.setTunings(config::ctlr::att::rate::P());
  success &= ctlr::att::rate::roll.setTunings(config::ctlr::att::rate::P());
  success &= ctlr::att::rate::yaw.setTunings(config::ctlr::att::rate::yawP(),
                                        config::ctlr::att::rate::yawI);
  ctlr::att::rate::yaw.setUpdateRate(config::ctlr::att::rate::yaw_updateRate);
  success &= ctlr::att::rate::pitch.setOutputLimits(
    config::ctlr::att::rate::outputLimit);
  success &= ctlr::att::rate::roll.setOutputLimits(
    config::ctlr::att::rate::outputLimit);
  success &= ctlr::att::rate::yaw.setOutputLimits(
    config::ctlr::att::rate::outputLimit);
  success &= ctlr::att::rate::yaw.setIntegralLimit(2);
  ctlr::att::rate::pitch.on(); ctlr::att::rate::roll.on(); ctlr::att::rate::yaw.on();

  success &= ctlr::att::angle::pitch.setTunings(config::ctlr::att::angle::P(),
    config::ctlr::att::angle::I());
  success &= ctlr::att::angle::roll.setTunings(config::ctlr::att::angle::P(),
    config::ctlr::att::angle::I());
  ctlr::att::angle::pitch.setUpdateRate(config::ctlr::att::angle::updateRate);
  ctlr::att::angle::roll.setUpdateRate(config::ctlr::att::angle::updateRate);
  success &= ctlr::att::angle::pitch.setOutputLimits(
    config::ctlr::att::angle::outputLimit);
  success &= ctlr::att::angle::roll.setOutputLimits(
    config::ctlr::att::angle::outputLimit);
  success &= ctlr::att::angle::pitch.setIntegralLimit(64);
  success &= ctlr::att::angle::roll.setIntegralLimit(64);
  ctlr::att::angle::pitch.on(); ctlr::att::angle::roll.on();

  ctlr::elev::rate.setTunings(0.012f, 0.02f, 0.0f);
  ctlr::elev::rate.setUpdateRate(37200);
  ctlr::elev::rate.setOutputLimits(-64, 64);
  ctlr::elev::rate.setIntegralLimit(5);
  ctlr::elev::rate.on();
  // ~Regulation initialization -----


  bmp.initialize();
  bmp.setPressureOversampleRatio(16); // 16
  bmp.setTemperatureOversampleRatio(1); // 2
  bmp.setFilterRatio(16); // 16
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
  float takeOffAltitude = calculateAltitude(p, seaLevelPressure, t);
  takeOffAltitude_mm = (int32_t)((takeOffAltitude + 0.5f) * 1000);

  #if DEBUG_TAKE_OFF_PRESSURE == true
  PRINT("Take off pressure / altitude: "); PRINT(takeOffPressure);
  PRINT(" / "); PRINT(takeOffAltitude_mm); PRINT("\n");
  #endif


  if (success == false) {
    PRINT("Wrong initialization configuration! Terminated.");
    exit(0);
  }
  indication::signal(ON);
  indication::warning(OFF);
  indication::lamp(config::indication::lamp());
  // indication::arms(0);

  // DEBUGLN("Setup done.\n----------");
} //void setup()


void loop() {
  static uint32_t usec_cycle_s = micros(); // used as a faster alternative

  #if PRINT_LOOP_PERIOD
  uart::print(micros() - usec_cycle_s); uart::print("\n");
  #endif

  #if DEBUG_LOOP_PERIOD
  DBG_PIN_TOGGLE();
  #endif

  usec_cycle_s = micros();


  // super_cycle - all cycles
  // cycle - one `loop` iteration
  // sub_cycle - doesn't exist here

  // TODO: joule count motor current


  /* Tasks performed every cycle: Get gyro and accel, rate ctlr, motor mix, indication
   * Tasks performed on specific cycle:
   * 0 - att fusion
   * 1 - comm, q->e, angle ctlr
   * 2 - read pressure
   * 3 - elev ctlr
   * 4 - read gps / black box
   */

  static int8_t cycle_s = -1;
  ++cycle_s; // start with 0

  if (cycle_s > 4) {
    cycle_s = 0;
  }


  /* Black box sub-cycles:
   * requires 210 bytes of RAM
   * 1 - write (1000us)
   *     use separate byte buffer
   * 2 - sync (700us)
   */
  // Black box buffer
  static uint8_t black_box_buffer[black_box_buffer_size];
  // since data is written to SD card every 2 full cycles, it contains:
  // gyro data: 2full_cycles * 5sub_cycles * 3axis * int16_t
  // acclrm data: 2full_cycles * 5sub_cycles * 3axis * int16_t
  // angular rate ctlr output: 2full_cycles * 5sub_cycles * 3axis * uint8_t
  // angle ctlr output: 2full_cycles * 1sub_cycles * 2axis * int32_t
  // rc input: 2full_cycles * 1sub_cycle * 8
  // throttle: 2full_cycles * 1sub_cycles * int8_t
  // altitude: 2full_cycles * 1sub_cycles * int16_t
  // elev ctlr output: 2full_cycles * 1sub_cycles * int16_t
  // = 192 bytes
  // data position is based on data type offset, cycle and super-cycle index
  // gyro data offset = 0
  // cycle 0, super-cycle 0: addr 0-5
  // cycle 1, super-cycle 0: addr 6-11
  // cycle 2, super-cycle 0: addr 12-17
  // cycle 3, super-cycle 0: addr 18-23
  // cycle 4, super-cycle 0: addr 24-29
  // cycle 0, super-cycle 1: addr 0-5 (+30)
  // cycle 1, super-cycle 1: addr 6-11 (+30)
  // cycle 2, super-cycle 1: addr 12-17 (+30)
  // cycle 3, super-cycle 1: addr 18-23 (+30)
  // cycle 4, super-cycle 1: addr 24-29 (+30)



  /* Get acceleration and angular rate
   * cycle: every
   * category: 'every-time (1/5)', 'angular rate control (1/3)'
   * timing: loose
   * cpu time: ~504usec @ 800kHz, ~700usec @ 400kHz
   * stack size: 13 bytes + TODO
   *
   * input: -
   * output: 3-axis acceleration, 3-axis angular rate
   * --------------------------------------------------------------------------
   */
  
  // buffer for storing the acceleration readings
  static Accel imu_accel_buffer[cfg::cycles_count];

  // angular rate is stored in a persistent variable in case the reading fails
  static AngularRate_mradps imu_angular_rate_mradps;


  { // scope the variables that will only be used for the current operation

    // count how many times in a row, reading the IMU has failed
    static uint32_t imu_data_staleness_cnt = 0;

    // Function `bool imu::Read(*data) {...}` is required to fill `data` with
    // 3-axis accelerometer and 3-axis gyroscope data in the order:
    // [0]: acceleration x
    // [1]: acceleration y
    // [2]: acceleration z
    // [3]: angular rate x
    // [4]: angular rate y
    // [5]: angular rate z
    // and return `false` if the read failed or `true` if it succeded.

    // temporary array that is used to get the readings from the IMU
    int16_t imu_data_tmp[6];
    if (imu::Read(imu_data_tmp)) {
      imu_data_staleness_cnt = 0;

      /* Acceleration data is used in 1 cycle as-is. Therefore it is buffered
       * based on the index of the current cycle and the index of the cycle
       * that will use it:
       */
      int8_t accel_buf_ndx_tmp = cycles::GetBufferIndex(cfg::accel_cycle,
        cycle_s, cfg::cycles_count);
      imu_accel_buf[accel_buf_ndx_tmp].x = imu_data_tmp[imu::accel::x::ndx];
      imu_accel_buf[accel_buf_ndx_tmp].y = imu_data_tmp[imu::accel::y::ndx];
      imu_accel_buf[accel_buf_ndx_tmp].z = imu_data_tmp[imu::accel::z::ndx];

      // Angular rate data is used in every cycle as milliradians per second:
      imu_angular_rate_mradps.pitch = imu_data_tmp[imu::gyro::x::ndx]
        * imu::raw_to_mradps;
      imu_angular_rate_mradps.roll = imu_data_tmp[imu::gyro::y::ndx]
        * imu::raw_to_mradps;
      imu_angular_rate_mradps.yaw = imu_data_tmp[imu::gyro::z::ndx]
        * imu::raw_to_mradps;

    } else { // unsuccessful IMU read
      ++imu_data_staleness_cnt;
      ++stat::imu::read_fails;

      // Populate next index in buffer with the data from the previous reading:
      int8_t accel_buf_ndx_tmp = cycles::GetBufferIndex(cfg::accel_cycle,
        cycle_s, cfg::cycles_count);
      imu_accel_buf[accel_buf_ndx_tmp].x = imu_accel_buf[accel_buf_ndx_tmp-1].x;
      imu_accel_buf[accel_buf_ndx_tmp].y = imu_accel_buf[accel_buf_ndx_tmp-1].y;
      imu_accel_buf[accel_buf_ndx_tmp].z = imu_accel_buf[accel_buf_ndx_tmp-1].z;
    } // imu::Read(...


  // Debugging ------------------------
  #if DBG_PRINT_IMU_ANGULAR_RATE_MRADPS

  static bool dbg_print_imu_angular_rate_mradps_units = false;
  if (dbg_print_imu_angular_rate_mradps_units == false) {
    dbg_print_imu_angular_rate_mradps_units = true;
    DBG_PRINT("pitch [mrad/s]\troll [mrad/s]\tyaw [mrad/s]\n");
  }

  DBG_PRINT(imu_angular_rate_mradps.pitch); PRINT("\t");
  DBG_PRINT(imu_angular_rate_mradps.roll); PRINT("\t");
  DBG_PRINT(imu_angular_rate_mradps.yaw); PRINT("\n");
  #endif


  #if DBG_PRINT_IMU_ACCELERATION

  static bool dbg_print_imu_acceleration_units = false;
  if (dbg_print_imu_angular_rate_mradps_units == false) {
    dbg_print_imu_angular_rate_mradps_units = true;
    DBG_PRINT("x\ty\tz\n");
  }

  int8_t accel_buf_ndx_tmp = cycles::GetBufferIndex(cfg::accel_cycle,
          cycle_s, cfg::cycles_count);

  DBG_PRINT(imu_accel_buf[accel_buf_ndx_tmp].x); PRINT("\t");
  DBG_PRINT(imu_accel_buf[accel_buf_ndx_tmp].y); PRINT("\t");
  DBG_PRINT(imu_accel_buf[accel_buf_ndx_tmp].z); PRINT("\n");
  #endif

  } // end of "Get acceleration and angular rate" task scope

  // Get acceleration and angular rate ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



  /* Angular rates controllers
   * cycle: every
   * category: 'every-time (2/5)', 'angular rate control (2/3)'
   * timing: exact
   * cpu time: ~210usecs
   *
   * input: angular rate SP | angular rate PV
   * output: motor difference expressed as pitch, roll and yaw
   */

  int8_t motor_pitch_diff = 0;
  int8_t motor_roll_diff = 0;
  int8_t motor_yaw_diff = 0;

  {
    static uint32_t angular_rate_ctlr_last_usec = usec_cycle_s;
    SYNC_CYCLE(angular_rate_ctlr_last_usec);
    angular_rate_ctlr_last_usec = micros();

    #if DEBUG_PERIOD_OF_ATT_RATE_CTLR
    DBG_PIN_TOGGLE();
    #endif

    // SP can be taken either from RC or the output of the angle ctlr
    // PV is always the angular rate
    // OP is always motor signal difference
    motor_pitch_diff = ctlr::att::rate::pitch.Compute(
      ctlr::att::rate::pitch::sp, imu_angular_rate_mradps.pitch);
    motor_roll_diff = ctlr::att::rate::roll.Compute(
      ctlr::att::rate::roll::sp, imu_angular_rate_mradps.roll);
    motor_yaw_diff = ctlr::att::rate::yaw.Compute(
      ctlr::att::rate::yaw::sp, imu_angular_rate_mradps.yaw);

    // ctlr::att::rate::yaw::op = ctlr::att::rate::yaw.Compute(
    //   ctlr::att::rate::yaw::sp, ctlr::att::rate::yaw::pv);
    // ctlr::att::rate::pitch::op = ctlr::att::rate::pitch.Compute(
    //   ctlr::att::rate::pitch::sp, ctlr::att::rate::pitch::pv);
    // ctlr::att::rate::roll::op = ctlr::att::rate::roll.Compute(
    //   ctlr::att::rate::roll::sp, ctlr::att::rate::roll::pv);

  }
  // Angular rates controllers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



  /* Throttle compensation
   * cycle: every
   * category: 'every-time (3/5)'
   * timing: loose
   * cpu time:
   *
   * input: throttle | motor difference expressed as pitch, roll and yaw
   * output: compensated throttle
   */


  /* Motor mix
   * cycle: every
   * category: 'every-time (4/5)', 'angular rate control (3/3)'
   * timing: loose
   * cpu time: ~20usec
   *
   * input: throttle | motor difference expressed as pitch, roll and yaw
   * output: motor control signal
   */

  motor::Set(motor_top_left,
             motor_top_right,
             motor_bottom_left,
             motor_bottom_right);


  uint8_t tl_p = 0;
  uint8_t tr_p = 0;
  uint8_t bl_p = 0;
  uint8_t br_p = 0;

  if (altHold) {
    throttle = 60;
    throttle += throttle_output;
  }

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
    ctlr::elev::rate.unwind();
  }
  OCR0B = 127 + tl_p;
  OCR1B = 127 + tr_p;
  OCR0A = 127 + bl_p;
  OCR1A = 127 + br_p;


  #if PRINT_THROTTLE
    if (!cycle_s % 2) {
      PRINT("Throttle: "); PRINT(throttle); PRINT("\n");
    }
  #endif

  #if PRINT_MOTOR_SIGNALS
    if (cycle_s == 0) {
      static uint32_t dm_lm = millis();
      if (millis() > dm_lm + 50) {
        dm_lm = millis();
        PRINT(OCR0B); PRINT("\t"); PRINT(OCR1B); PRINT("\n");
        PRINT(OCR0A); PRINT("\t"); PRINT(OCR1A); PRINT("\n");
        uint8_t i = 0;
        while (i++ < 12) PRINT("\n");
      }
    }
  #endif
  // ~Motor mix ---



  /* Indication
   * cycle: every
   * category: 'every-time (5/5)'
   * timing: exact
   *
   * input: system status
   * output: indication control signal
   */



  // Sub-tasks --------------

  /* Filter accelerometer data
   * cycle: 1 x x x x
   * category: 'angle control (1/4)'
   * timing: loose
   *
   * input: series of 3-axis acceleration
   * output: 3-axis acceleration
   */
  static Accel imu_accel;
  if (cycle_s == 0) {
    imu_accel.x = MedianFilter5(imu_accel_buffer[0].x, imu_accel_buffer[1].x,
      imu_accel_buffer[2].x, imu_accel_buffer[3].x, imu_accel_buffer[4].x);
    imu_accel.y = MedianFilter5(imu_accel_buffer[0].y, imu_accel_buffer[1].y,
      imu_accel_buffer[2].y, imu_accel_buffer[3].y, imu_accel_buffer[4].y);
    imu_accel.z = MedianFilter5(imu_accel_buffer[0].z, imu_accel_buffer[1].z,
      imu_accel_buffer[2].z, imu_accel_buffer[3].z, imu_accel_buffer[4].z);
  }


  /* Attitude fusion
   * cycle: 1 x x x x
   * category: 'angle control (2/4)'
   * timing: exact
   *
   * input: 3-axis acceleration | 3-axis angular rate | 3-axis magnetic field
   * output: orientation quaternion
   */
  if (cycle_s == 0) {
    fusion::Compute(
      imu_accel.x,
      imu_accel.y,
      imu_accel.y,
      imu_angular_rate_mradps.x,
      imu_angular_rate_mradps.y,
      imu_angular_rate_mradps.z
      );
  }


  /* Quaternion to Euler
   * cycle: x 2 x x x
   * category: 'angle control (3/4)'
   * timing: loose
   *
   * input: orientation quaternion
   * output: pitch, roll and yaw angle PV
   */
  static Attitude_mradps attitude_mradps;
  if (cycle_s == 1) {
    // fusion::
  }


  /* Angle controllers
   * cycle: x 2 x x x
   * category: 'angle control (4/4)' 
   * timing: exact
   *
   * input: pitch, roll and yaw angle SP | pitch, roll and yaw angle PV
   * output: pitch, roll and yaw angular rate SP
   */


  /* Communication
   * cycle: x 2 x x x
   * category: 'communication (1/2)'
   * timing: loose
   *
   * input: -
   * output: flight mode | 4-axis position
   */


  /* User input processing and shaping
   * cycle: x 2 x x x
   * category: 'communication (2/2)'
   * timing: loose
   *
   * input: flight mode | 4-axis position
   * output: throttle / elev rate SP | angular rate SP / angle SP
   */


  #if TASK_READ_PRESSURE
  /* Read pressure
   * cycle: x x 3 x x
   * category: 'elevation control (1/2)'
   * timing: loose
   *
   * input: -
   * output: atmospheric pressure
   */

  #endif


  #if TASK_ELEV_CTL
  /* Elev ctlr
   * cycle: x x x 4 x
   * category: 'elevation control (2/2)'
   * timing: exact
   *
   * input: elevation rate SP | elevation rate PV
   * output: throttle correction
   */

  #endif


  #if TASK_EXTRA
  /* Free (gps/bb)
   * cycle: x x x x 5
   * timing: loose
   *
   * input:
   * output:
   */

  #endif




  /* RF
   *
   */
  comm::Task();


  /* Shape RF command
   *
   */
  if (comm::command_s.FlightMode == FlightMode::stab) {
    ctlr::att::angle::pitch::sp = comm::command_s.y;
    ctlr::att::angle::roll::sp = comm::command_s.x;
  } else if (comm::command_s.FlightMode == FlightMode::acro) {
    ctlr::att::rate::pitch::sp = comm::command_s.y;
    ctlr::att::rate::roll::sp = comm::command_s.x;
  }








  /* Angle controller
   * sub-cycle: 0 x x x
   * timing: exact
   * cpu time: ~240usec
   */
  if (cycle_s == 0) {
    static uint32_t angle_ctlr_last_usec_s = usec_cycle_s;
    SYNC_SUB_CYCLE(angle_ctlr_last_usec_s);
    angle_ctlr_last_usec_s = micros();

    #if DEBUG_ANGLE_CTRL_PERIOD == true
    DBG_PIN_TOGGLE();
    #endif

    // SP is taken from RC
    // PV is fused angle
    // OP is angular rate SP
    ctlr::att::angle::pitch::op = ctlr::att::angle::pitch.Compute(
      ctlr::att::angle::pitch::sp, ctlr::att::angle::pitch::pv);
    ctlr::att::angle::roll::op = ctlr::att::angle::roll.Compute(
      ctlr::att::angle::roll::sp, ctlr::att::angle::roll::pv);

    // ctlr::att::angle::pitch.compute();
    // ctlr::att::angle::roll.compute();


    #if PRINT_CTRL_PITCH
    if (command.flightMode == FlightMode::stabilize) {
      PRINT(command.pitch);
      PRINT(","); PRINT(attitude.pitch);
    } else {
      PRINT(","); PRINT(ctlr::att::rate::sp::pitch);
      PRINT(","); PRINT(angularVelocity.y);
    }
    PRINT(","); PRINT(ctlr::att::rate::output::pitch);
    PRINT("\n");
    #endif

    #if PRINT_CTRL_ROLL
    if (command.flightMode == FlightMode::stabilize) {
      PRINT(command.roll);
      PRINT(","); PRINT(attitude.roll);
    }
    PRINT(","); PRINT(ctlr::att::rate::sp::roll);
    PRINT(","); PRINT(angularVelocity.x);
    PRINT(","); PRINT(ctlr::att::rate::output::roll);
    PRINT("\n");
    #endif

    #if PRINT_CTRL_YAW
    PRINT(command.yaw);
    PRINT(", "); PRINT(avYaw);
    PRINT(", "); PRINT(ctlr::att::rate::output::yaw);
    PRINT("\n");
    #endif
  }
  // ~Angle controller ---


  /* Fusion
   * sub-cycle: x x x 3
   * timing: exact
   * cpu time: ~1900usec (fuse: ~1000usec, quaternion to euler: ~680usec)
   */

  #if FUSION_COMPLEMENTARY_FILTER
    float x_sqr = square((float)acceleration.x);
    float y_sqr = square((float)acceleration.y);
    float z_sqr = square((float)acceleration.z);

    float roll_a = atan2( (float) - acceleration.x,
                          sqrt(y_sqr + z_sqr) )
                   * toDegrees;
    float pitch_a = atan2( (float) acceleration.y,
      sign(acceleration.z) * sqrt(z_sqr + (config::imu::epsilon * x_sqr)))
                    * toDegrees;

    static uint32_t fusion_last_usec_s = usec_cycle_s;
    uint32_t usec_now = micros();
    float fusion_dt_sec = (float)(usec_now - fusion_last_usec_s) / 1000000.0f;
    fusion_last_usec_s = usec_now;

    attitude.pitch = (attitude.pitch + angularVelocity.x * fusion_dt_sec) * config::imu::complementary::alpha()
                     + pitch_a * config::imu::complementary::oneMinusAlpha;
    attitude.roll = (attitude.roll + angularVelocity.y * fusion_dt_sec) * config::imu::complementary::alpha()
                    + roll_a * config::imu::complementary::oneMinusAlpha;

    #if PRINT_ATTITUDE_ACCEL
      if (cycle_s == 0) {
        DEBUG(pitch_a); DEBUG(", "); DEBUG(angularVelocity.x);
        DEBUG(", ");  DEBUG(roll_a); DEBUG(", "); DEBUGLN(angularVelocity.y);
      }
    #endif

  #elif FUSION_MAHONY_FILTER
    static uint32_t fusion_last_usec_s = usec_cycle_s;
    SYNC_CYCLE(fusion_last_usec_s);
    fusion_last_usec_s = micros();

    mahony.updateIMU(angularVelocity.x, angularVelocity.y, angularVelocity.z,
                     acceleration.x, acceleration.y, acceleration.z);


    attitude.roll = mahony.getPitch();
    attitude.pitch = mahony.getRoll();
    float yaw_mahony = mahony.getYaw();


    #if ANGULAR_RATE_DIRECT_FROM_GYROSCOPE
      avYaw = angularVelocity.z;

    #elif ANGULAR_RATE_DIFFERENTIATE_FROM_MAHONY
      static uint32_t diff_angular_rate_last_usec_s = usec_cycle_s;

      static float yaw_mahony_last_s = yaw_mahony;
      uint32_t usec_now = micros();  
      uint32_t yaw_dt_sec = (float)(usec_now - diff_angular_rate_last_usec_s)
        / 1000000.0f;
      avYaw = (yaw_mahony - yaw_mahony_last_s) / yaw_dt_sec;
      yaw_mahony_last_s = yaw_mahony;
      diff_angular_rate_last_usec_s = usec_now;

    #endif // angular rate

  #endif // fusion


  #if PRINT_ATTITUDE
    if (sub_cycle_s == 0) {
      PRINT(attitude.pitch); PRINT(", ");
      PRINT(attitude.roll); PRINT("\n");
    }
  #endif
  // ~Fusion ---


  /* RF
   * timing: loose
   * cpu time: ~230usec, no packet: ~32usec
   */
  static uint32_t command_last_usec_s = usec_cycle_s;
  static uint32_t batt_volt_last_usec_s = usec_cycle_s;

  if (sub_cycle_s == 0 && comm::rf24.available()) {
    while (comm::rf24.available()) {
      uint8_t rawMessage[32];
      comm::rf24.read(&rawMessage, 32);

      switch ((PacketType)rawMessage[0]) {
      case PacketType::Command:
        memcpy(&command, &rawMessage, sizeof(command));
        handleCommand();
        command_last_usec_s = usec_cycle_s;
        break;
      case PacketType::Setting:
        Setting setting;
        memcpy(&setting, &rawMessage, sizeof(setting));
        handleSetting(setting);
        break;
      default:
        comm::rf24.reset();
        PRINT("unrec "); PRINT(rawMessage[0]); PRINT("\n");
        break;
      }
    }

    #if PRINT_COMMAND
    if (command.flightMode == FlightMode::stabilize) {
      PRINT("STAB - ");
    } else if (command.flightMode == FlightMode::acro) {
      PRINT("ACRO - ");
    } else {
      PRINT("DIRECT - ");
    }
    PRINT("Th: "); PRINT(command.throttle);
    PRINT(", P: "); PRINT(command.pitch);
    PRINT(", R: "); PRINT(command.roll);
    PRINT(", Y: "); PRINT(command.yaw);
    PRINT("\n");
    #endif

  } else if (usec_cycle_s - command_last_usec_s > config::communication::commandTimeout) {
    throttle = 0;
    ctlr::att::angle::pitch.on();
    ctlr::att::angle::roll.on();
    command.pitch = 0;
    command.roll = 0;
    command.yaw = 0;
    altHold = false;
    ctlr::elev::rate.unwind();
    throttle_output = 0;
    status::communication = Status::error;

  } else if (usec_cycle_s - batt_volt_last_usec_s > config::battery::updateRate) {
    batt_volt_last_usec_s = usec_cycle_s;
    batteryVoltage = readBatteryVoltage();
    static float batteryVoltage_last = 0.0;
    batteryVoltage = lowPassFilter(batteryVoltage, batteryVoltage_last, config::battery::alpha);
    batteryVoltage_last = batteryVoltage;

    #if PRINT_BATTERY_VOLTAGE
      PRINT("Vb: "); PRINT(batteryVoltage); PRINT("\n");
    #endif

    if (batteryVoltage > config::battery::lowVoltage) {
      status::battery = Status::normal;
    } else if (batteryVoltage > config::battery::criticalVoltage) {
      status::battery = Status::warning;
    } else {
      status::battery = Status::error;
    }
  } else if (battery_throttle_update == true) {
    static bool out_of_ee_space = false;
    static bool write_millis_pending = false;
    static uint32_t out_of_space_millis = 0;

    if (out_of_ee_space && write_millis_pending) {
      if (eeprom_is_ready()) {
        write_millis_pending = false;

        eeprom_update_dword(1005, out_of_space_millis);
        // DEBUG("done at "); DEBUG(out_of_space_millis);
        // DEBUG("\n");
      }
    }

    if (out_of_ee_space == false && eeprom_is_ready()) {
      battery_throttle_update = false;

      static uint16_t ee_addr = EE_FIRST_FREE_ADDRESS;

      eeprom_write_block(&battery_throttle_buffer, ee_addr,
        sizeof(battery_throttle_buffer));

      // DEBUG("wrote: "); DEBUG(battery_throttle_buffer.battery);
      // DEBUG(", "); DEBUG(battery_throttle_buffer.throttle);
      // DEBUG("\n");
      battery_throttle_buffer.battery = 321;
      battery_throttle_buffer.throttle = 123;

      indication::arms(0);

      if (ee_addr + 3 < 1000) {
        ee_addr += 3;
      } else {
        out_of_ee_space = true;
        write_millis_pending = true;
        out_of_space_millis = millis();
      }
    }

  }
  // ~RF ---


  // Pressure -----
  static float pressure = takeOffPressure;
  static float pressure_prev = takeOffPressure;
  static float temperature = 0.0f;
  static float pressure_raw[3] = {
    takeOffPressure, takeOffPressure, takeOffPressure
  };
  static uint8_t pressure_ndx = 0;

  if (sub_cycle_s == 2) { // 730usec @ 800kHz, 850usec @ 400kHz

    #if REPORT_TIMINGS
    timings_g[NDX_READ_BMP][0] = micros();
    #endif
    bmp.read(pressure_raw[pressure_ndx], temperature);
    #if REPORT_TIMINGS
    timings_g[NDX_READ_BMP][timings_it] = micros() - timings_g[NDX_READ_BMP][0];
    #endif

  } else if (sub_cycle_s == 3) { // 400us ?
    if (pressure_ndx >= 2) {
      pressure_ndx = 0;
      pressure = middle_of_3(
        pressure_raw[0], pressure_raw[1], pressure_raw[2]
      );
    } else {
      ++pressure_ndx;
    }

    // if (pressure < pressure_prev - 5000.0f
    //   || pressure > pressure + 5000.0f) {
    //   pressure = pressure_prev;
    // }

    if (pressure_ndx == 0) {
      // ~620usec

      pressure = pressure * 0.05f + pressure_prev * 0.95f;
      pressure_prev = pressure;

      #if DEBUG_PRESSURE == true
      PRINT("Pressure: "); PRINT(pressure); PRINT("hPa\n");
      #endif

      float absAlt_m = calculateAltitude(pressure,
        seaLevelPressure, temperature);
      int32_t absAlt_mm = (int32_t)((absAlt_m + 0.5f) * 1000);
      int32_t relAlt_mm = absAlt_mm - takeOffAltitude_mm;

      #if DEBUG_ALTITUDE == true
      if (pressure_ndx == 0) {
        PRINT("Alt: "); PRINT(absAlt_mm); PRINT("mm / ");
        PRINT(relAlt_mm); PRINT("mm\n");
      }
      #endif

      // Vertical speed
      static int32_t relAltLast_mm = relAlt_mm;
      static uint32_t vertSpeedLUS = usec_cycle_s;
      // float vertSpeedDelta_s = (float)(micros() - vertSpeedLUS) / 1000000.0f;
      int16_t vertSpeedFreq_hz = 1000000 / (micros() - vertSpeedLUS);
      vertSpeedLUS = micros();

      // int16_t vertSpeed_mm_s = (relAlt_mm - relAltLast_mm) / vertSpeedDelta_s;
      vertSpeed_mmps = (relAlt_mm - relAltLast_mm) * vertSpeedFreq_hz;
      relAltLast_mm = relAlt_mm;

      vertSpeed_input_mmps = vertSpeed_mmps;

      // vertSpeedInput_mm_s = vertSpeed_mm_s; // float = int16_t

      #if PRINT_VERTICAL_SPEED
      // PRINT("Speed: "); PRINT(vertSpeed_mmps); PRINT("mm/s\n");
      PRINT("Speed: "); PRINT((vertSpeed_mmps+0.5f) / 1000); PRINT("m/s\n");
      #endif

      if (altHold == true) {
        ctlr::elev::rate.compute();
      }

      #if DEBUG_VERT_SPEED_ctlr == true
      PRINT("VertSpeed SP: "); PRINT(vertSpeed_sp_mmps); PRINT(", ");
      PRINT("T: "); PRINT(throttle_output); PRINT("\n");
      #endif

      // ctlr::vertAccel.compute();

      #if BLACK_BOX_HOVER_VOLTAGE
      static bool hover_voltage_enabled = true;


      // Check if near hover
      // TODO: must save values for different battery voltages
      if ((hover_voltage_enabled == true) && (batteryVoltage > 6.0f)) {
        if ( (vertSpeed_input_mmps + VERT_SPEED_HOVER_THRESHOLD > 0)
          || (vertSpeed_input_mmps - VERT_SPEED_HOVER_THRESHOLD < 0) ) {
          
          uint16_t batt_mv = (batteryVoltage + 0.5f) * 1000;
          static uint16_t batt_last_mv = batt_mv;

          if ((batt_mv + 35 < batt_last_mv) || (batt_mv - 35 > batt_last_mv)) {
            battery_throttle_buffer.battery = batt_mv;
            battery_throttle_buffer.throttle = throttle;
            battery_throttle_update = true;

            batt_last_mv = batt_mv;

          } // if significant voltage change
        } // if vert speed is low
      } // if hover voltage is enabled and on battery
      #endif
    }
  }
  // ~Pressure -----


  // Indication
  // 8usec
  if (sub_cycle_s == 1) {

    if (status::communication == Status::normal) {
      if (indication::arms() != expMap[clamp(config::indication::armsLevel(), 0, 13)]) {
        uint8_t armsPwm = indication::arms();
        if (armsPwm > expMap[clamp(config::indication::armsLevel(), 0, 13)]) {
          indication::arms(--armsPwm);
        } else if (armsPwm < expMap[clamp(config::indication::armsLevel(), 0, 13)]) {
          indication::arms(++armsPwm);
        }
      }
    }

    static uint32_t indication_last_usec_s = usec_cycle_s;
    if (usec_cycle_s - indication_last_usec_s > config::indication::period) {
      indication_last_usec_s = usec_cycle_s;
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

    altHold = false;
    altHold_enTime = 0;
    ctlr::elev::rate.off();

  } else {
    if (command.flightMode == FlightMode::stabilize) {
      ctlr::att::angle::pitch.setMode(AUTOMATIC);
      ctlr::att::angle::roll.setMode(AUTOMATIC);
    } else if (command.flightMode == FlightMode::acro) {
      ctlr::att::angle::pitch.setMode(MANUAL);
      ctlr::att::angle::roll.setMode(MANUAL);
      ctlr::att::rate::sp::pitch = command.pitch;
      ctlr::att::rate::sp::roll = command.roll;
      altHold = false;
      throttle_output = 0;
    } else {
      ctlr::att::rate::pitch.off();
      ctlr::att::rate::roll.off();
      ctlr::att::angle::pitch.off();
      ctlr::att::angle::roll.off();
      ctlr::elev::rate.off();
      altHold = false;
      throttle_output = 0;
    }


    throttle = command.throttle;

    if (command.altitudeHold && command.throttle > 5) {

      altHold = true;
      altHold_enTime = micros();
      ctlr::elev::rate.on();

      // Transform incoming throttle command to vert speed sp
      #define RF_THROTTLE_BOT (0)
      #define RF_THROTTLE_TOP (127)
      // 127 - 0 = 127 / 2 = 63 + 0 = 63
      #define RF_THROTTLE_MID ( (((RF_THROTTLE_TOP) - (RF_THROTTLE_BOT)) / 2) + (RF_THROTTLE_BOT))

      #define VERT_SPEED_MIN_MPS (-3)
      #define VERT_SPEED_MAX_MPS (3)
      #define VERT_SPEED_MIN_MMPS ((VERT_SPEED_MIN_MPS) * 1000)
      #define VERT_SPEED_MAX_MMPS ((VERT_SPEED_MAX_MPS) * 1000)

      #define VERT_SPEED_DEAD_BAND (15)

      // (63 - 15) * x = 3000
      // x = 3000 / 48 = 62
      #define VERT_SPEED_SP_MULT ( (VERT_SPEED_MAX_MMPS) / ((RF_THROTTLE_MID) - (VERT_SPEED_DEAD_BAND)) )


      int16_t th_cen = command.throttle - RF_THROTTLE_MID;
      if (th_cen > VERT_SPEED_DEAD_BAND) { // ascend
        vertSpeed_sp_mmps = (th_cen - VERT_SPEED_DEAD_BAND) * VERT_SPEED_SP_MULT;
      } else if (th_cen < (VERT_SPEED_DEAD_BAND * -1)) { // descend
        vertSpeed_sp_mmps = (th_cen + VERT_SPEED_DEAD_BAND) * VERT_SPEED_SP_MULT;
      } else { // don't move
        vertSpeed_sp_mmps = 0.0f;
      }

    } else {
      altHold = false;
      altHold_enTime = 0;
      ctlr::elev::rate.off();
      ctlr::elev::rate.unwind();
      throttle = command.throttle;
      throttle_output = 0;
    }

    if (command.flightMode == FlightMode::acro) {
      altHold = false;
      altHold_enTime = 0;
      ctlr::elev::rate.off();
      ctlr::elev::rate.unwind();
      throttle = command.throttle;
      throttle_output = 0;
    }

    status::communication = Status::normal;
  }
}

void handleSetting(Setting & setting) {
  setting.success = true;
  switch (setting.id) {
  case SettingId::dummy:
    setting.success = false;
    PRINT("dummy");
    break;

  // case SettingId::imuLpf_common:
  //   if (setting.request) {
  //     setting.value = (float)config::imu::lowPassFilter::common();
  //     DEBUGLN("Requested LPF");
  //   } else {
  //     setting.success = config::imu::lowPassFilter::common.changeValue(setting.value);
  //     setting.value = (float)config::imu::lowPassFilter::common();
  //     imu::mpu.setDLPFMode(config::imu::lowPassFilter::common());
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
  // case SettingId::regInner_outputLimit:
  //   if (setting.request) {
  //     DEBUGLN("Requested Output limit (angular_rate)");
  //     setting.value = (float)config::ctlr::att::rate::outputLimit();
  //   } else {
  //     setting.success = config::ctlr::att::rate::outputLimit.changeValue(setting.value);
  //     setting.value = (float)config::ctlr::att::rate::outputLimit();
  //     ctlr::att::rate::pitch.setOutputLimits(config::ctlr::att::rate::outputLimit());
  //     ctlr::att::rate::roll.setOutputLimits(config::ctlr::att::rate::outputLimit());
  //     ctlr::att::rate::yaw.setOutputLimits(config::ctlr::att::rate::outputLimit());
  //     DEBUG("Output limit (angular_rate) changed to "); DEBUGLN(setting.value);
  //   }
  //   break;
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
  // case SettingId::regOuter_updateRate:
  //   if (setting.request) {
  //     DEBUGLN("Requested angle update rate");
  //     setting.value = (float)config::ctlr::att::angle::updateRate();
  //   } else {
  //     setting.success = config::ctlr::att::angle::updateRate.changeValue(setting.value);
  //     setting.value = (float)config::ctlr::att::angle::updateRate();
  //     ctlr::att::angle::pitch.setUpdateRate(config::ctlr::att::angle::updateRate());
  //     ctlr::att::angle::roll.setUpdateRate(config::ctlr::att::angle::updateRate());
  //     DEBUG("angle update rate changed to "); DEBUGLN(setting.value);
  //   }
  //   break;
  // case SettingId::regOuter_outputLimit:
  //   if (setting.request) {
  //     DEBUGLN("Requested angle output limit");
  //     setting.value = (float)config::ctlr::att::angle::outputLimit();
  //   } else {
  //     setting.success = config::ctlr::att::angle::outputLimit.changeValue(setting.value);
  //     setting.value = (float)config::ctlr::att::angle::outputLimit();
  //     ctlr::att::rate::pitch.setOutputLimits(config::ctlr::att::angle::outputLimit());
  //     ctlr::att::rate::roll.setOutputLimits(config::ctlr::att::angle::outputLimit());
  //     ctlr::att::rate::yaw.setOutputLimits(config::ctlr::att::angle::outputLimit());
  //     DEBUG("angle output limit changed to "); DEBUGLN(setting.value);
  //   }
  //   break;
  // case SettingId::reg_minRegThrottle:
  //   if (setting.request) {
  //     DEBUGLN("Requested Min reg throttle");
  //     setting.value = (float)config::ctlr::att::minimumRegulationThrottle();
  //   } else {
  //     setting.success = config::ctlr::att::minimumRegulationThrottle.changeValue(setting.value);
  //     setting.value = (float)config::ctlr::att::minimumRegulationThrottle();
  //     DEBUG("Min reg throttle changed to "); DEBUGLN(setting.value);
  //   }
  //   break;
  // case SettingId::reg_maxBaseThrottle:
  //   if (setting.request) {
  //     DEBUGLN("Requested Max base throttle");
  //     setting.value = (float)config::ctlr::att::maximumBaseThrottle();
  //   } else {
  //     setting.success = config::ctlr::att::maximumBaseThrottle.changeValue(setting.value);
  //     setting.value = (float)config::ctlr::att::maximumBaseThrottle();
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
  // case SettingId::indication_lamp:
  //   if (setting.request) {
  //     DEBUGLN("?Lamp");
  //     setting.value = (float)config::indication::lamp();
  //   } else {
  //     setting.success = config::indication::lamp.changeValue(setting.value);
  //     setting.value = (float)config::indication::lamp();
  //     indication::lamp(config::indication::lamp());
  //     DEBUG("Lamp>"); DEBUGLN(setting.value);
  //   }
  //   break;
  // case SettingId::regAlt_p:
  //   if (setting.request) {
  //     DEBUGLN("?AltP");
  //     setting.value = config::ctlr::altitude::P();
  //   } else {
  //     setting.success = config::ctlr::altitude::P.changeValue(setting.value);
  //     setting.value = config::ctlr::altitude::P();
  //     verticalVelocity_ctlr.setTunings(setting.value, config::ctlr::altitude::P(), 0.0f, config::ctlr::altitude::D());
  //     DEBUG("AltP>"); DEBUGLN(setting.value);
  //   }
  //   break;
  // case SettingId::regAlt_d:
  //   if (setting.request) {
  //     DEBUGLN("?AltD");
  //     setting.value = config::ctlr::altitude::D();
  //   } else {
  //     setting.success = config::ctlr::altitude::D.changeValue(setting.value);
  //     setting.value = config::ctlr::altitude::D();
  //     verticalVelocity_ctlr.setTunings(setting.value, config::ctlr::altitude::P(), 0.0f, config::ctlr::altitude::D());
  //     DEBUG("AltD>"); DEBUGLN(setting.value);
  //   }
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
