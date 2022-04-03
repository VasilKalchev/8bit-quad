#include "src/config/config.hpp"
#include "src/util/debug.hpp"
#include "src/board/8bit-quad-fc_board.hpp"

#include "src/peripheral/ADC.hpp"
#include <UART_atmega328.hpp>

#include "src/fusion/MahonyAHRS.h"
#include "src/control/PID.hpp"
#include "src/util/utils.hpp"

#include "src/rx/rx.hpp"
#include "src/config/config_rx.hpp"



#if defined(IMU_MPU6050)
  #include "src/imu/MPU6050.hpp"
#elif defined(IMU_MPU925x)
  #include "src/imu/MPU9255.hpp"
#endif


enum class k_FlightMode : uint8_t {
  Angle, Acro, Direct,
};

enum State : uint8_t {
  k_On = 1, k_Off = 0, k_FullOn = 255, k_HalfOn = 127,
};

enum class k_Status {
  normal, warning, error,
};


// Global instances
rx::State_t g_rx_state;
k_FlightMode g_flight_mode;
Mahony g_mahony;

namespace imu {
#if defined(IMU_MPU6050)
  MPU6050_i2cDev g_mpu(cfg::imu::i2c_address);
#elif defined(IMU_MPU925x)
  MPU9255 g_mpu(cfg::imu::i2c_address);
#endif
} //namespace imu

// Declared in src/imu/imu_types.hpp
AngularVelocity g_angular_velocity;
Acceleration g_acceleration;
Attitude g_attitude;


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

      PID pitch(&sp::pitch, &g_angular_velocity.x, &output::pitch);
      PID roll(&sp::roll, &g_angular_velocity.y, &output::roll);
      PID yaw(&g_rx_state.yaw, &g_angular_velocity.z, &output::yaw);
    } // namespace rate

    namespace angle {
      PID pitch(&g_rx_state.pitch, &g_attitude.pitch, &rate::sp::pitch);
      PID roll(&g_rx_state.roll, &g_attitude.roll, &rate::sp::roll);
    } // namespace angle

  } // namespace att
} // namespace ctlr


namespace status {
  k_Status communication = k_Status::normal;
  k_Status battery = k_Status::normal;
} // namespace status

float g_battery_voltage = 0.0;


#define SYNC_SUB_CYCLE(LUS) while (micros() - LUS < cfg::schedule::sub_cycle_time);
#define SYNC_CYCLE(LUS) while (micros() - LUS < cfg::schedule::cycle_time);


void setup() {
  bool success = true;

  replaceTimer0WithTimer2();

  UART_INIT(cfg::uart_baud_rate);
  PRINT(F("\nRST---\nMAD 0 RC\n\n"));

  cfg::init();

  initIO();
  indication::signal(k_Off);
  indication::warning(k_On);
  indication::arms(k_Off);

  Wire.begin();
  Wire.setClock(cfg::imu::clock_speed); // 800kHz
  // TWBR = 12;  // 400 kHz (maximum)

  DBG_PIN_INIT();


  PRINT(F("CFG ---\n"));

  PRINT(F(" ANGLE ctlr\n"));
  PRINT(F("  P = ")); PRINT(cfg::ctlr::att::angle::P);
  PRINT(F("\tI = ")); PRINT(cfg::ctlr::att::angle::I); PRINT(F("\n\n"));

  PRINT(F(" RATE ctlr\n"));
  PRINT(F("  tilt: P = ")); PRINT(cfg::ctlr::att::rate::P);
  PRINT(F("\tI = ")); PRINT(cfg::ctlr::att::rate::I);
  PRINT(F("\tD = ")); PRINT(cfg::ctlr::att::rate::D); PRINT(F("\n"));
  PRINT(F("  yaw: P = ")); PRINT(cfg::ctlr::att::rate::yaw::P);
  PRINT(F("\tI = ")); PRINT(cfg::ctlr::att::rate::yaw::I); PRINT(F("\n\n"));

  PRINT(F("Max input\n"));
  PRINT(F("throttle: ")); PRINT(1000 / cfg::rx::div::throttle);
  PRINT(F("\nangle: ")); PRINT(500 / cfg::rx::div::angle_tilt);
  PRINT(F("\nrate: ")); PRINT(500 / cfg::rx::div::rate_tilt);
  PRINT(F("\nrate yaw: ")); PRINT(500 / cfg::rx::div::rate_yaw);
  PRINT(F("\nraw tilt: ")); PRINT(500 / cfg::rx::div::tilt);
  PRINT(F("\nraw yaw: ")); PRINT(500 / cfg::rx::div::yaw);
  PRINT(F("\n\n"));

  PRINT(F("^^^\n"));

  initADC();

  // IMU initialization
  PRINT(F("IMU "));
  success &= imu::g_mpu.initialize();
  if (success) { PRINT(F("done")); } else { PRINT(F("fail")); }
  PRINT(F("\n"));
  // PRINT(F(". T = ")); PRINT(imu::g_mpu.getTemperature()); PRINT(F("degC.\n"));
  // ~IMU initialization -----

  g_mahony.begin(384);

  // Regulation initialization
  PRINT(F("Regulation "));
  success &= ctlr::att::rate::pitch.setTunings(
    cfg::ctlr::att::rate::P,
    cfg::ctlr::att::rate::I,
    cfg::ctlr::att::rate::D
    );
  success &= ctlr::att::rate::roll.setTunings(
    cfg::ctlr::att::rate::P,
    cfg::ctlr::att::rate::I,
    cfg::ctlr::att::rate::D
    );
  success &= ctlr::att::rate::yaw.setTunings(
    cfg::ctlr::att::rate::yaw::P,
    cfg::ctlr::att::rate::yaw::I
    );
  ctlr::att::rate::yaw.setUpdateRate(cfg::ctlr::att::rate::yaw::update_rate);
  success &= ctlr::att::rate::pitch.setOutputLimits(
    cfg::ctlr::att::rate::output_limit);
  success &= ctlr::att::rate::roll.setOutputLimits(
    cfg::ctlr::att::rate::output_limit);
  success &= ctlr::att::rate::yaw.setOutputLimits(
    cfg::ctlr::att::rate::output_limit);
  success &= ctlr::att::rate::yaw.setIntegralLimit(
    cfg::ctlr::att::rate::yaw::integral_limit);
  ctlr::att::rate::pitch.on();
  ctlr::att::rate::roll.on();
  ctlr::att::rate::yaw.on();

  success &= ctlr::att::angle::pitch.setTunings(
    cfg::ctlr::att::angle::P,
    cfg::ctlr::att::angle::I
    );
  success &= ctlr::att::angle::roll.setTunings(
    cfg::ctlr::att::angle::P,
    cfg::ctlr::att::angle::I
    );
  ctlr::att::angle::pitch.setUpdateRate(cfg::ctlr::att::angle::update_rate);
  ctlr::att::angle::roll.setUpdateRate(cfg::ctlr::att::angle::update_rate);
  success &= ctlr::att::angle::pitch.setOutputLimits(
    cfg::ctlr::att::angle::output_limit);
  success &= ctlr::att::angle::roll.setOutputLimits(
    cfg::ctlr::att::angle::output_limit);
  success &= ctlr::att::angle::pitch.setIntegralLimit(
    cfg::ctlr::att::angle::integral_limit);
  success &= ctlr::att::angle::roll.setIntegralLimit(
    cfg::ctlr::att::angle::integral_limit);
  ctlr::att::angle::pitch.on();
  ctlr::att::angle::roll.on();

  if (success) PRINT(F("done")); else PRINT(F("fail"));
  PRINT(F("\n"));
  // ~Regulation initialization -----

  if (success == false) {
    PRINT(F("Wrong init config!"));
    exit(0);
  }

  indication::signal(k_On);
  indication::warning(k_Off);
  indication::lamp(cfg::indication::lamp);

  sei(); // enable interrupts

  PRINT(F("Setup done.\n---"));
} //void setup()


void loop() {
  /* Component: Microseconds for this cycle
   * Variable that stores the microseconds since power-on. It is updated at the
   * beginning of the cycle. Can be used as a liter alternative for
   * non-critical timing.
   */
  static uint32_t usec_cycle_s = micros(); // used as a faster alternative


  /* Component: Sub-cycle
   * Sub-cycle index.
   */
  static int8_t sub_cycle_s = -1;
  ++sub_cycle_s;

  // keep sub-cycle index in range
  if ((sub_cycle_s >= (uint8_t)cfg::schedule::SubCycle_ndx::OUT_OF_RANGE) || (sub_cycle_s < (uint8_t)cfg::schedule::SubCycle_ndx::FIRST)) {
    sub_cycle_s = (uint8_t)cfg::schedule::SubCycle_ndx::FIRST;
  }


  #if PRINT_LOOP_PERIOD
  PRINT(sub_cycle_s); PRINT(F("\t"));
  PRINT(micros() - usec_cycle_s); PRINT(F("\n"));
  #endif

  #if DEBUG_LOOP_PERIOD
  DBG_PIN_TOGGLE();
  #endif

  usec_cycle_s = micros();


  /* Angular rates controllers
   * timing: exact
   * schedule: 4/4 (every sub-cycle)
   * cpu time: ~350usec
   */
  static uint32_t angular_rate_ctlr_last_usec_s = usec_cycle_s;
  SYNC_SUB_CYCLE(angular_rate_ctlr_last_usec_s);
  #if PRINT_RATE_CTRL_PERIOD
  PRINT(micros() - angular_rate_ctlr_last_usec_s); PRINT(F("\n"));
  #endif
  #if DEBUG_RATE_CTRL_PERIOD
  DBG_PIN_TOGGLE();
  #endif

  angular_rate_ctlr_last_usec_s = micros();
  ctlr::att::rate::yaw.compute();
  ctlr::att::rate::pitch.compute();
  ctlr::att::rate::roll.compute();
  // ~Angular rate controller ---


  /* Angle controller
   * timing: exact
   * schedule: 1/4 (every 4 cycles)
   * cpu time: ~240usec
   */
  if (sub_cycle_s == (uint8_t)cfg::schedule::SubCycle_ndx::angleCtrl) {
    static uint32_t angle_ctlr_last_usec_s = usec_cycle_s;
    SYNC_CYCLE(angle_ctlr_last_usec_s);
    #if PRINT_ANGLE_CTRL_PERIOD
    PRINT(micros() - angle_ctlr_last_usec_s); PRINT(F("\n"));
    #endif
    #if DEBUG_ANGLE_CTRL_PERIOD
    DBG_PIN_TOGGLE();
    #endif

    angle_ctlr_last_usec_s = micros();
    ctlr::att::angle::pitch.compute();
    ctlr::att::angle::roll.compute();


    #if PRINT_CTRL_PITCH
    if (g_flight_mode == k_FlightMode::Acro) {
      PRINT(F(",")); PRINT(ctlr::att::rate::sp::pitch);
      PRINT(F(",")); PRINT(g_angular_velocity.y);
    } else {
      PRINT(g_rx_state.pitch);
      PRINT(F(",")); PRINT(g_attitude.pitch);
    }
    PRINT(F(",")); PRINT(ctlr::att::rate::output::pitch);
    PRINT(F("\n"));
    #endif

    #if PRINT_CTRL_ROLL
    if (g_flight_mode == k_FlightMode::Acro) {
      PRINT(F(",")); PRINT(ctlr::att::rate::sp::roll);
      PRINT(F(",")); PRINT(g_angular_velocity.x);
    } else {
      PRINT(g_rx_state.roll);
      PRINT(F(",")); PRINT(g_attitude.roll);
    }
    PRINT(F(",")); PRINT(ctlr::att::rate::output::roll);
    PRINT(F("\n"));
    #endif

    #if PRINT_CTRL_YAW
    PRINT(g_rx_state.yaw);
    PRINT(F(", ")); PRINT(g_angular_velocity.z);
    PRINT(F(", ")); PRINT(ctlr::att::rate::output::yaw);
    PRINT(F("\n"));
    #endif
  }
  // ~Angle controller ---


  /* Read IMU
   * timing: loose
   * schedule: 4/4 (every cycle)
   * cpu time: ~530usec @ 800kHz, ~650usec @ 500kHz, ~740usec @ 400kHz
   */
  uint8_t imu_rd_st = imu::g_mpu.getMotion(&g_angular_velocity, &g_acceleration);
  #if DEBUG
  static uint16_t rd_fails = 0;
  static uint16_t zeroes = 0;
  if (imu_rd_st == 1) {
    PRINT(F("IMU blank: ")); PRINT(zeroes++); PRINT(F("\n"));
  } else if (imu_rd_st == 2) {
    PRINT(F("IMU fails: ")); PRINT(rd_fails++); PRINT(F("\n"));
  }
  #endif

  // g_angular_velocity.z += 0.05; // trim yaw

  // if (cfg::imu::median_filter_acc == true) {
  //   static uint8_t acc_ndx = 0;
  //   static Acceleration acc_f;
  //   static Acceleration acc_buff[2];
  //   if (acc_ndx < 2) {
  //     acc_buff[acc_ndx].x = g_acceleration.x;
  //     acc_buff[acc_ndx].y = g_acceleration.y;
  //     acc_buff[acc_ndx].z = g_acceleration.z;
  //     ++acc_ndx;
  //   } else {
  //     acc_ndx = 0;
  //     acc_f.x = util::Median(acc_buff[0].x,
  //       acc_buff[1].x, g_acceleration.x);
  //     acc_f.y = util::Median(acc_buff[0].y,
  //       acc_buff[1].y, g_acceleration.y);
  //     acc_f.z = util::Median(acc_buff[0].z,
  //       acc_buff[1].z, g_acceleration.z);
  //   }

  //   g_acceleration.x = acc_f.x;
  //   g_acceleration.y = acc_f.y;
  //   g_acceleration.z = acc_f.z;
  // }


  #if PRINT_ANGULAR_RATE
  PRINT(g_angular_velocity.x); PRINT(F(", "));
  PRINT(g_angular_velocity.y); PRINT(F(", "));
  PRINT(g_angular_velocity.z); PRINT(F(", "));
  // static float heading = 0;
  // heading += g_angular_velocity.z;
  // PRINT(heading);
  PRINT(F("\n"));
  #endif

  #if PRINT_ACCELERATION
  PRINT(g_acceleration.x); PRINT(F(", "));
  PRINT(g_acceleration.y); PRINT(F(", "));
  PRINT(g_acceleration.z - 16384); PRINT(F("\n"));
  #endif
  // ~Read IMU ---


  /* Fusion
   * timing: exact
   * schedule: 4/4 (every cycle)
   * cpu time: 
   *  complementary: N/A
   *  Mahony: fuse 1000usec + quat to p+r: 680usec
   */
  // Complementary filter
  #if ATTITUDE_FUSION_METHOD_COMPLEMENTARY
  float x_sqr = square((float)g_acceleration.x);
  float y_sqr = square((float)g_acceleration.y);
  float z_sqr = square((float)g_acceleration.z);

  float roll_a = atan2( (float) - g_acceleration.x,
    sqrt(y_sqr + z_sqr) )
  * util::toDegrees;
  float pitch_a = atan2( (float) g_acceleration.y,
    util::sign(g_acceleration.z) * sqrt(z_sqr + (cfg::imu::epsilon * x_sqr)))
  * util::toDegrees;

  static uint32_t fusion_last_usec_s = usec_cycle_s;
  uint32_t usec_now = micros();
  float fusion_dt_sec = (float)(usec_now - fusion_last_usec_s) / 1000000.0f;
  fusion_last_usec_s = usec_now;

  g_attitude.pitch = (g_attitude.pitch + g_angular_velocity.x * fusion_dt_sec) * cfg::imu::complementary::alpha()
  + pitch_a * cfg::imu::complementary::oneMinusAlpha;
  g_attitude.roll = (g_attitude.roll + g_angular_velocity.y * fusion_dt_sec) * cfg::imu::complementary::alpha()
  + roll_a * cfg::imu::complementary::oneMinusAlpha;

  // Mahony
  #elif ATTITUDE_FUSION_METHOD_MAHONY
  static uint32_t fusion_last_usec_s = usec_cycle_s;
  SYNC_SUB_CYCLE(fusion_last_usec_s);
  #if PRINT_FUSION_PERIOD
  PRINT(micros() - fusion_last_usec_s); PRINT(F("\n"));
  #endif
  #if DEBUG_FUSION_PERIOD
  DBG_PIN_TOGGLE();
  #endif

  fusion_last_usec_s = micros();
  g_mahony.updateIMU(g_angular_velocity.x, g_angular_velocity.y, g_angular_velocity.z,
   g_acceleration.x, g_acceleration.y, g_acceleration.z);
  #endif // mahony
  // ~Fusion ---


  /* Quaternion to Euler
   * timing: loose
   * schedule: 1/4 (every 4 cycles)
   * cpu time: ~680usec
   */
  #if ATTITUDE_FUSION_METHOD_MAHONY
  if (sub_cycle_s == (uint8_t)cfg::schedule::SubCycle_ndx::quat2euler) {
    #if PRINT_QUAT2EULER_PERIOD
    PRINT(micros() - quat2euler_last_usec_s); PRINT(F("\n"));
    #endif
    #if DEBUG_QUAT2EULER_PERIOD
    DBG_PIN_TOGGLE();
    #endif

    g_attitude.roll = g_mahony.getPitch();
    g_attitude.pitch = g_mahony.getRoll();
  }
  #endif // fusion method = Mahony

  #if PRINT_ATTITUDE
  if (sub_cycle_s == (uint8_t)cfg::schedule::SubCycle_ndx::quat2euler) {
    PRINT(g_attitude.pitch); PRINT(F(", "));
    PRINT(g_attitude.roll); PRINT(F("\n"));
  }
  #endif


  /* Motor mix
   * timing: loose
   * schedule: 4/4 (every cycle)
   * cpu time: ~20usec
   */
  uint8_t tl_p = 0;
  uint8_t tr_p = 0;
  uint8_t bl_p = 0;
  uint8_t br_p = 0;

  if (g_rx_state.throttle > cfg::mix::minimum_regulation_throttle) {
    if (g_rx_state.throttle > cfg::mix::maximum_base_throttle) {
      g_rx_state.throttle = cfg::mix::maximum_base_throttle;
    }

    #if COMPENSATE_THROTTLE_WHEN_TILTING
    int16_t throttle_output = g_rx_state.throttle;

    float tilt = abs(g_attitude.pitch) + abs(g_attitude.roll);
    if (tilt > 45.0) tilt = 45.0;
    float compensation = 1 + (tilt / 45.0);
    throttle_output *= compensation;

    #else
    int16_t throttle_output = g_rx_state.throttle;
    #endif


    tl_p = util::clamp(throttle_output
     + ctlr::att::rate::output::pitch + ctlr::att::rate::output::roll - ctlr::att::rate::output::yaw,
     5, 127);
    tr_p = util::clamp(throttle_output
     + ctlr::att::rate::output::pitch - ctlr::att::rate::output::roll + ctlr::att::rate::output::yaw,
     5, 127);
    bl_p = util::clamp(throttle_output
     - ctlr::att::rate::output::pitch + ctlr::att::rate::output::roll + ctlr::att::rate::output::yaw,
     5, 127);
    br_p = util::clamp(throttle_output
     - ctlr::att::rate::output::pitch - ctlr::att::rate::output::roll - ctlr::att::rate::output::yaw,
     5, 127);
  } else {
    tl_p = g_rx_state.throttle;
    tr_p = g_rx_state.throttle;
    bl_p = g_rx_state.throttle;
    br_p = g_rx_state.throttle;
    ctlr::att::angle::pitch.unwind();
    ctlr::att::angle::roll.unwind();
    ctlr::att::rate::yaw.unwind();
  }

  OCR0B = 127 + tl_p;
  OCR1B = 127 + tr_p;
  OCR0A = 127 + bl_p;
  OCR1A = 127 + br_p;


  #if PRINT_THROTTLE
  PRINT(g_rx_state.throttle); PRINT(F("\n"));
  #endif

  #if PRINT_MOTOR_SIGNALS
  PRINT(OCR0B); PRINT(F("\t")); PRINT(OCR1B); PRINT(F("\n"));
  PRINT(OCR0A); PRINT(F("\t")); PRINT(OCR1A); PRINT(F("\n\n"));
  #endif
  // ~Motor mix ---


  /* Battery voltage
   * timing: loose
   * schedule: 1/4 (every 4 cycles)
   * cpu time: 160usec
   */
  if (sub_cycle_s == (uint8_t)cfg::schedule::SubCycle_ndx::battery) {
    static uint32_t batt_volt_last_usec_s = 0;
    if (usec_cycle_s - batt_volt_last_usec_s > cfg::battery::update_rate) {
      batt_volt_last_usec_s = usec_cycle_s;

      g_battery_voltage = readBatteryVoltage();
      static float battery_voltage_last = 0.0;
      g_battery_voltage = util::lowPassFilter(g_battery_voltage, battery_voltage_last, cfg::battery::alpha);
      battery_voltage_last = g_battery_voltage;

      #if PRINT_BATTERY_VOLTAGE
      PRINT(F("Vb: ")); PRINT(g_battery_voltage); PRINT(F("\n"));
      #endif

      if (g_battery_voltage > cfg::battery::low_voltage) {
        status::battery = k_Status::normal;
      } else if (g_battery_voltage > cfg::battery::critical_voltage) {
        status::battery = k_Status::warning;
      } else {
        status::battery = k_Status::error;
      }
    }
  }


  /* Indication
   * timing: loose
   * schedule: 1/4 (every 4 cycles)
   * cpu time: 8usec
   */
  #define NEW_INDICATION (0)
  #if NEW_INDICATION == 1
  // run every cycle
  // SW PWM lamp and PCB LEDs
  // indicate current mode (on initial switch or permanently)
  // proper fade

  if (g_flight_mode == k_FlightMode::stabilize) {

  }

  arms_pwm_level = indication::arms();
  if (arms_pwm_level < cfg::indication::arms_level) {
    indication::arms(++arms_pwm_level);
  } else if (arms_pwm_level > cfg::indication::arms_level) {
    indication::arms(--arms_pwm_level);
  }

  #else // OLD_INDICATION
  if (sub_cycle_s == (uint8_t)cfg::schedule::SubCycle_ndx::indication) {
    if (status::communication == k_Status::normal) {
      if (indication::arms() != util::expMap[util::clamp(cfg::indication::arms_level, 0, 13)]) {
        uint8_t armsPwm_current = indication::arms();
        if (armsPwm_current > util::expMap[util::clamp(cfg::indication::arms_level, 0, 13)]) {
          indication::arms(--armsPwm_current);
        } else if (armsPwm_current < util::expMap[util::clamp(cfg::indication::arms_level, 0, 13)]) {
          indication::arms(++armsPwm_current);
        }
      }
    }

    static uint32_t indication_last_usec_s = usec_cycle_s;
    if (usec_cycle_s - indication_last_usec_s > cfg::indication::period) {
      indication_last_usec_s = usec_cycle_s;

      if (status::battery == k_Status::normal && status::communication == k_Status::normal) {
        indication::toggleSignal();
        indication::warning(0);
      } else if (status::communication != k_Status::normal) {
        indication::toggleSignal();
        indication::warning(!indication::signal());
        indication::toggleArms();
      } else if (status::battery == k_Status::warning) {
        indication::toggleSignal();
        indication::warning(!indication::signal());
      } else if (status::battery == k_Status::error) {
        indication::signal(0);
        indication::toggleWarning();
      } else {
        status::communication = k_Status::normal;
        status::battery = k_Status::normal;
      }
    }
  }
  #endif
  // ~Indication -----


  /* RC
   * timing: loose
   * schedule: 1/4 (every 4 cycles)
   * cpu time: 150usec, old: 76usec
   */
  if (sub_cycle_s == (uint8_t)cfg::schedule::SubCycle_ndx::rc) {
    // Read raw values
    int16_t rc_raw[(uint8_t)cfg::rx::k_Ch::Count] = { 0 };

    rx::ReadRx(pin::communication::ino::ppm,
      rc_raw, (uint8_t)cfg::rx::k_Ch::Count);

    #if PRINT_RC_RAW
    static uint8_t print_rc_raw_div = 0;
    if (++print_rc_raw_div > 1) {
      print_rc_raw_div = 0;

      PRINT(F("t ")); PRINT(rc_raw[(uint8_t)cfg::rx::k_Ch::Throttle]);
      PRINT(F("\tp ")); PRINT(rc_raw[(uint8_t)cfg::rx::k_Ch::Pitch]);
      PRINT(F("\tr ")); PRINT(rc_raw[(uint8_t)cfg::rx::k_Ch::Roll]);
      PRINT(F("\ty ")); PRINT(rc_raw[(uint8_t)cfg::rx::k_Ch::Yaw]);
      PRINT(F("\tab ")); PRINT(rc_raw[(uint8_t)cfg::rx::k_Ch::SwAB]);
      PRINT(F("\tcd ")); PRINT(rc_raw[(uint8_t)cfg::rx::k_Ch::SwCD]);
      PRINT(F("\tpA ")); PRINT(rc_raw[(uint8_t)cfg::rx::k_Ch::VrA]);
      PRINT(F("\tpB ")); PRINT(rc_raw[(uint8_t)cfg::rx::k_Ch::VrB]);
      PRINT(F("\n"));
    }
    #endif


    for (uint8_t ndx = 0; ndx < cfg::rx::sticks_count; ++ndx) {
      // clamp sticks' input between min (1000) and max (2000)
      util::clamp(rc_raw[ndx], cfg::rx::input::min, cfg::rx::input::max);

      if ((cfg::rx::k_Ch)ndx == cfg::rx::k_Ch::Throttle) {
        // change range for throttle input from 1000..2000 to 0..1000
        rc_raw[ndx] -= cfg::rx::input::min;
      } else {
        // change range for sticks' input from 1000..2000 to -500..500
        rc_raw[ndx] -= cfg::rx::input::mid;
      }
    }

    g_rx_state.throttle = rc_raw[(uint8_t)cfg::rx::k_Ch::Throttle]
      / cfg::rx::div::throttle;


    int16_t switches[4] = { 0 };

    // Convert to switch state
    #if RC_SW_A
    PRINT(F("RC_SW_A no\n"));
    switches[0] = rc_raw[(uint8_t)cfg::rx::k_Ch::SwA];
    #elif RC_SW_B
    PRINT(F("RC_SW_B no\n"));
    switches[1] = rc_raw[(uint8_t)cfg::rx::k_Ch::SwB];
    #elif RC_SW_AB
    rx::SplitChannel(rc_raw[(uint8_t)cfg::rx::k_Ch::SwAB], switches[0], 2, switches[1], 2);
    #endif

    #if RC_SW_C
    PRINT(F("RC_SW_C no\n"));
    switches[2] = rc_raw[(uint8_t)cfg::rx::k_Ch::SwC];
    #elif RC_SW_D
    PRINT(F("RC_SW_D no\n"));
    switches[3] = rc_raw[(uint8_t)cfg::rx::k_Ch::SwD];
    #elif RC_SW_CD
    rx::SplitChannel(rc_raw[(uint8_t)cfg::rx::k_Ch::SwCD], switches[2], 3, switches[3], 2);
    #endif

    #if PRINT_RC_SWITCHES_STATE
    PRINT(F("a ")); PRINT(switches[0]); PRINT(F("\tb ")); PRINT(switches[1]);
    PRINT(F("\tc ")); PRINT(switches[2]); PRINT(F("\td ")); PRINT(switches[3]);
    PRINT(F("\n"));
    #endif


    static int16_t vr_a_last = rc_raw[(uint8_t)cfg::rx::k_Ch::VrA];
    static int16_t vr_b_last = rc_raw[(uint8_t)cfg::rx::k_Ch::VrB];

    int16_t vr_a_diff = rc_raw[(uint8_t)cfg::rx::k_Ch::VrA] - vr_a_last;
    int16_t vr_b_diff = rc_raw[(uint8_t)cfg::rx::k_Ch::VrB] - vr_b_last;

    #if PRINT_RC_POTENTIOMETERS
    PRINT(F("vrA: ")); PRINT(rc_raw[(uint8_t)cfg::rx::k_Ch::VrA]); PRINT(F("\n"));
    PRINT(F("a_last: ")); PRINT(vr_a_last); PRINT(F("\n"));
    PRINT(F("a_diff: ")); PRINT(vr_a_diff); PRINT(F("\n"));
    #endif


    uint16_t sw_map = rx::CombineChannels(switches, 4);
    static uint16_t rc_state_last = sw_map;
    bool change = false;
    if (sw_map != rc_state_last) {
      rc_state_last = sw_map;
      change = true;
    }

    #if PRINT_RC_SWITCHES_MASK
    PRINT(sw_map); PRINT(F("\n"));
    for (uint8_t sw = 0; sw < 4; ++sw) {
      for (uint8_t st = 0; st < 3; ++st) {
        if (((sw_map >> (st + (3 * sw))) & 1) == 1) {
          PRINT(sw+1); PRINT(st+1); PRINT(F(" "));
        }
      }
    }
    PRINT(F("\n"));
    #endif
    
    if ((sw_map & cfg::rx::switch_state::flight_mode::angle) == cfg::rx::switch_state::flight_mode::angle) {
      if (change) {
        PRINT(F("RC_ST_FM_ANGLE\n"));
      }

      g_flight_mode = k_FlightMode::Angle;
      status::communication = k_Status::normal;

      g_rx_state.pitch = rc_raw[(uint8_t)cfg::rx::k_Ch::Pitch]
        / cfg::rx::div::angle_tilt;
      g_rx_state.roll = rc_raw[(uint8_t)cfg::rx::k_Ch::Roll]
        / cfg::rx::div::angle_tilt;
      g_rx_state.yaw = rc_raw[(uint8_t)cfg::rx::k_Ch::Yaw]
        / cfg::rx::div::rate_yaw;
    }

    if ((sw_map & cfg::rx::switch_state::flight_mode::acro) == cfg::rx::switch_state::flight_mode::acro) {
      if (change) {
        PRINT(F("RC_ST_FM_ACRO\n"));
      }

      g_flight_mode = k_FlightMode::Acro;
      status::communication = k_Status::normal;

      g_rx_state.pitch = rc_raw[(uint8_t)cfg::rx::k_Ch::Pitch]
        / cfg::rx::div::rate_tilt;
      g_rx_state.roll = rc_raw[(uint8_t)cfg::rx::k_Ch::Roll]
        / cfg::rx::div::rate_tilt;
      g_rx_state.yaw = rc_raw[(uint8_t)cfg::rx::k_Ch::Yaw]
        / cfg::rx::div::rate_yaw;
    }

    if ((sw_map & cfg::rx::switch_state::flight_mode::direct) == cfg::rx::switch_state::flight_mode::direct) {
      if (change) {
        PRINT(F("RC_ST_FM_DIRECT\n"));
      }

        g_flight_mode = k_FlightMode::Direct;
        status::communication = k_Status::error;

        g_rx_state.pitch = rc_raw[(uint8_t)cfg::rx::k_Ch::Pitch]
          / cfg::rx::div::tilt;
        g_rx_state.roll = rc_raw[(uint8_t)cfg::rx::k_Ch::Roll]
          / cfg::rx::div::tilt;
        g_rx_state.yaw = rc_raw[(uint8_t)cfg::rx::k_Ch::Yaw]
          / cfg::rx::div::yaw;
    }

    if ((sw_map & cfg::rx::switch_state::cfg::lights) == cfg::rx::switch_state::cfg::lights) {
      if (change) {
        PRINT(F("RC_CFG_LIGHTS\n"));
      }

      if (vr_a_diff > cfg::rx::pot_step) {
        // inc arms LEDs
        vr_a_last = rc_raw[(uint8_t)cfg::rx::k_Ch::VrA];
        PRINT(F("inc arms LEDs\n"));

      } else if (vr_a_diff < -cfg::rx::pot_step) {
        // dec arms LEDs
        vr_a_last = rc_raw[(uint8_t)cfg::rx::k_Ch::VrA];
        PRINT(F("dec arms LEDs\n"));

      }

      if (vr_b_diff > cfg::rx::pot_step) {
        // inc lamp
        vr_b_last = rc_raw[(uint8_t)cfg::rx::k_Ch::VrB];
        PRINT(F("inc lamp\n"));

      } else if (vr_b_diff < -cfg::rx::pot_step) {
        // dec lamp
        vr_b_last = rc_raw[(uint8_t)cfg::rx::k_Ch::VrB];
        PRINT(F("dec lamp\n"));

      }
    }

    if ((sw_map & cfg::rx::switch_state::calibration::accel) == cfg::rx::switch_state::calibration::accel) {
      if (change) {
        PRINT(F("RC_ST_CAL_ACCEL\n"));
      }

      static uint8_t calib_accel_div = 0;
      ++calib_accel_div;
      if (calib_accel_div > 10) {
        calib_accel_div = 0;

        if (g_rx_state.throttle > 2 && g_flight_mode == k_FlightMode::Angle) {
          #if defined(IMU_MPU6050)
          cfg::imu::offset::accel[1] -= (g_rx_state.pitch / 5);
          imu::g_mpu.setYAccelOffset(cfg::imu::offset::accel[1]);
          cfg::imu::offset::accel[0] += (g_rx_state.roll / 5);
          imu::g_mpu.setXAccelOffset(cfg::imu::offset::accel[0]);
          #elif defined(IMU_MPU925x)
          cfg::imu::offset::accel[1] += (g_rx_state.pitch / 5);
          imu::g_mpu.setYAccelOffset(cfg::imu::offset::accel[1]);
          cfg::imu::offset::accel[0] -= (g_rx_state.roll / 5);
          imu::g_mpu.setXAccelOffset(cfg::imu::offset::accel[0]);
          #endif
        }
        PRINT(F("xa: ")); PRINT(cfg::imu::offset::accel[0]);
        PRINT(F("\n"));
        PRINT(F("ya: ")); PRINT(cfg::imu::offset::accel[1]);
        PRINT(F("\n\n"));
      } // calib_div
    }

    if ((sw_map & cfg::rx::switch_state::cfg::accel_lpf) == cfg::rx::switch_state::cfg::accel_lpf) {
      if (change) {
        PRINT(F("RC_CFG_ACCEL_LPF\n"));
      }
      
    }

    if ((sw_map & cfg::rx::switch_state::cfg::mahony) == cfg::rx::switch_state::cfg::mahony) {
      if (change) {
        PRINT(F("RC_CFG_MAHONY\n"));
      }
      
    }

    if ((sw_map & cfg::rx::switch_state::calibration::gyro) == cfg::rx::switch_state::calibration::gyro) {
      if (change) {
        PRINT(F("RC_ST_CAL_GYRO\n"));
      }

      if (g_rx_state.throttle < 40 && g_flight_mode == k_FlightMode::Acro) {
        static float gyroBuff[3][3] = {0};
        static int8_t gyroBuff_ndx = -1;

        ++gyroBuff_ndx;
        if (gyroBuff_ndx < 3) {
          gyroBuff[gyroBuff_ndx][0] = g_angular_velocity.x;
          gyroBuff[gyroBuff_ndx][1] = g_angular_velocity.y;
          gyroBuff[gyroBuff_ndx][2] = g_angular_velocity.z;
        } else {
          gyroBuff_ndx = 0;

          float x = 0;
          float y = 0;
          float z = 0;
          for (uint8_t i = 0; i < 3; ++i) {
            x += gyroBuff[i][0];
            y += gyroBuff[i][1];
            z += gyroBuff[i][2];
          }
          x /= 3;
          y /= 3;
          z /= 3;
          PRINT(F("avg: "));
          PRINT(x); PRINT(F(", "));
          PRINT(y); PRINT(F(", "));
          PRINT(z); PRINT(F("\n"));

          if (x < -0.05) {
            cfg::imu::offset::gyro[0] += 1;
            imu::g_mpu.setXGyroOffset(cfg::imu::offset::gyro[0]);
            PRINT(F("inc x\n"));
          } else if (x > 0.05) {
            cfg::imu::offset::gyro[0] -= 1;
            imu::g_mpu.setXGyroOffset(cfg::imu::offset::gyro[0]);
            PRINT(F("dec x\n"));
          }
          if (y < -0.05) {
            cfg::imu::offset::gyro[1] += 1;
            imu::g_mpu.setYGyroOffset(cfg::imu::offset::gyro[1]);
            PRINT(F("inc y\n"));
          } else if (y > 0.05) {
            cfg::imu::offset::gyro[1] -= 1;
            imu::g_mpu.setYGyroOffset(cfg::imu::offset::gyro[1]);
            PRINT(F("dec y\n"));
          }
          if (z < -0.05) {
            cfg::imu::offset::gyro[2] += 1;
            imu::g_mpu.setZGyroOffset(cfg::imu::offset::gyro[2]);
            PRINT(F("inc z\n"));
          } else if (z > 0.05) {
            cfg::imu::offset::gyro[2] -= 1;
            imu::g_mpu.setZGyroOffset(cfg::imu::offset::gyro[2]);
            PRINT(F("dec z\n"));
          }
          PRINT(F("offsets: "));
          PRINT(cfg::imu::offset::gyro[0]); PRINT(F(", "));
          PRINT(cfg::imu::offset::gyro[1]); PRINT(F(", "));
          PRINT(cfg::imu::offset::gyro[2]);
          PRINT(F("\n"));
        }
      }
    }

    if ((sw_map & cfg::rx::switch_state::cfg::imu_lpf) == cfg::rx::switch_state::cfg::imu_lpf) {
      if (change) {
        PRINT(F("RC_CFG_IMU_LPF\n"));
      }
      
    }

    if ((sw_map & cfg::rx::switch_state::cfg::angle_p) == cfg::rx::switch_state::cfg::angle_p) {
      if (change) {
        PRINT(F("RC_CFG_ANGLE_P\n"));
      }
      
    }

    if ((sw_map & cfg::rx::switch_state::cfg::angle_i) == cfg::rx::switch_state::cfg::angle_i) {
      if (change) {
        PRINT(F("RC_CFG_ANGLE_I\n"));
      }
      
    }

    if ((sw_map & cfg::rx::switch_state::cfg::rate_p) == cfg::rx::switch_state::cfg::rate_p) {
      if (change) {
        PRINT(F("RC_CFG_RATE_P\n"));
      }
      
    }

    if ((sw_map & cfg::rx::switch_state::cfg::rate_d) == cfg::rx::switch_state::cfg::rate_d) {
      if (change) {
        PRINT(F("RC_CFG_RATE_D\n"));
      }
      
    }

    if ((sw_map & cfg::rx::switch_state::cfg::yaw_rate_p) == cfg::rx::switch_state::cfg::yaw_rate_p) {
      if (change) {
        PRINT(F("RC_CFG_YAW_P\n"));
      }
      
    }

    if ((sw_map & cfg::rx::switch_state::cfg::yaw_rate_i) == cfg::rx::switch_state::cfg::yaw_rate_i) {
      if (change) {
        PRINT(F("RC_CFG_YAW_I\n"));
      }
      
    }



    static k_FlightMode flight_mode_last = k_FlightMode::Angle;
    if (flight_mode_last != g_flight_mode) {
      flight_mode_last = g_flight_mode;
      indication::arms(3);
    }


    #if PRINT_RC
    PRINT(F("t: ")); PRINT(g_rx_state.throttle);
    PRINT(F(" p: ")); PRINT(g_rx_state.pitch);
    PRINT(F(" r: ")); PRINT(g_rx_state.roll);
    PRINT(F(" y: ")); PRINT(g_rx_state.yaw);
    PRINT(F(" fm: ")); PRINT((uint8_t)g_flight_mode);
    PRINT(F("\n"));
    #endif
  }
  // ~RC


  if (g_flight_mode == k_FlightMode::Angle) {
    ctlr::att::angle::pitch.setMode(AUTOMATIC);
    ctlr::att::angle::roll.setMode(AUTOMATIC);

    ctlr::att::rate::pitch.setMode(AUTOMATIC);
    ctlr::att::rate::roll.setMode(AUTOMATIC);
    ctlr::att::rate::yaw.setMode(AUTOMATIC);

  } else if (g_flight_mode == k_FlightMode::Acro) {
    ctlr::att::angle::pitch.setMode(MANUAL);
    ctlr::att::angle::roll.setMode(MANUAL);

    ctlr::att::rate::pitch.setMode(AUTOMATIC);
    ctlr::att::rate::roll.setMode(AUTOMATIC);
    ctlr::att::rate::yaw.setMode(AUTOMATIC);

    ctlr::att::rate::sp::pitch = g_rx_state.pitch;
    ctlr::att::rate::sp::roll = g_rx_state.roll;

  } else if (g_flight_mode == k_FlightMode::Direct) {
    ctlr::att::angle::pitch.setMode(MANUAL);
    ctlr::att::angle::roll.setMode(MANUAL);

    ctlr::att::rate::pitch.setMode(MANUAL);
    ctlr::att::rate::roll.setMode(MANUAL);
    ctlr::att::rate::yaw.setMode(MANUAL);

    ctlr::att::rate::output::pitch = g_rx_state.pitch;
    ctlr::att::rate::output::roll = g_rx_state.roll;
    ctlr::att::rate::output::yaw = g_rx_state.yaw;

  } else {
    // ctlr::att::rate::pitch.off();
    // ctlr::att::rate::roll.off();
    // ctlr::att::angle::pitch.off();
    // ctlr::att::angle::roll.off();
  }

} //void loop()
