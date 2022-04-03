/* Remote controller user configuration.
 *
 */

#pragma once


namespace cfg {
namespace rx {

constexpr uint8_t sticks_count = 4;
#define RC_NONE (0)  // const
#define RC_STICKS_MODE_CUSTOM (0)  // const

#define RC_STICKS_MODE (2)

#define RC_SW_A (RC_NONE)
#define RC_SW_B (RC_NONE)
#define RC_SW_C (RC_NONE)
#define RC_SW_D (RC_NONE)
#define RC_SW_AB (5)
#define RC_SW_CD (6)
#define RC_VR_A (7)
#define RC_VR_B (8)
#define RC_SNR (RC_NONE)
#define RC_ERROR (RC_NONE)
#define RC_PPM1 (RC_NONE)
#define RC_PPM2 (RC_NONE)
#define RC_PPM3 (RC_NONE)

#if RC_SW_AB && (RC_SW_A | RC_SW_B)
  #error "RC: invalid combination of A, B and A+B"
#endif
#if RC_SW_CD && (RC_SW_C | RC_SW_D)
  #error "RC: invalid combination of C, D and C+D"
#endif


namespace input {
constexpr int16_t min = 1000;
constexpr int16_t max = 2000;
constexpr int16_t mid = (min + max) / 2;

constexpr uint8_t channels = 8;
} // namespace input

/* RC channels
 */
enum class k_Ch : uint8_t {
#if RC_STICKS_MODE == 1
  RHoriz = 0, Roll = 0,
  LVert = 1, Throttle = 1,
  RVert = 2, Pitch = 2,
  LHoriz = 3, Yaw = 3,
#elif RC_STICKS_MODE == 2
  RHoriz = 0, Roll = 0,
  RVert = 1, Pitch = 1,
  LVert = 2, Throttle = 2,
  LHoriz = 3, Yaw = 3,
#elif RC_STICKS_MODE == 3
  LHoriz = 0, Yaw = 0,
  LVert = 1, Throttle = 1,
  RVert = 2, Pitch = 2,
  RHoriz = 3, Roll = 3,
#elif RC_STICKS_MODE == 4
  LHoriz = 0, Yaw = 0,
  RVert = 1, Pitch = 1,
  LVert = 2, Throttle = 2,
  RHoriz = 3, Roll = 3,
#elif RC_STICKS_MODE == (RC_STICKS_MODE_CUSTOM)
  LVert = 0, Throttle = 0,
  LHoriz = 1, Yaw = 1,
  RVert = 2, Pitch = 2,
  RHoriz = 3, Roll = 3,
#else
  #error "Unknown RC_STICKS_MODE!"
#endif

#if RC_SW_A
  SwA = RC_SW_A-1,
#endif
#if RC_SW_B
  SwB = RC_SW_B-1,
#endif
#if RC_SW_C
  SwC = RC_SW_C-1,
#endif
#if RC_SW_D
  SwD = RC_SW_D-1,
#endif
#if RC_SW_AB
  SwAB = RC_SW_AB-1,
#endif
#if RC_SW_CD
  SwCD = RC_SW_CD-1,
#endif
#if RC_VR_A
  VrA = RC_VR_A-1,
#endif
#if RC_VR_B
  VrB = RC_VR_B-1,
#endif
#if RC_SNR
  SNR = RC_SNR-1,
#endif
#if RC_ERROR
  ERROR = RC_ERROR-1,
#endif
#if RC_PPM1
  PPM1 = RC_PPM1-1,
#endif
#if RC_PPM2
  PPM2 = RC_PPM2-1,
#endif
#if RC_PPM3
  PPM3 = RC_PPM3-1,
#endif
  Count,
};


/* Map functionality to switches state
 */

// Constants
// enum class k_SwitchStateMap : uint16_t {
//     A_min = (1 << 0), A_mid = (1 << 1), A_max = (1 << 2),
//     B_min = (1 << 3), B_mid = (1 << 4), B_max = (1 << 5),
//     C_min = (1 << 6), C_mid = (1 << 7), C_max = (1 << 8),
//     D_min = (1 << 9), D_mid = (1 << 10), D_max = (1 << 11),
// };

namespace switch_state {
namespace a {
constexpr uint16_t min = (1 << 0);
constexpr uint16_t mid = (1 << 1);
constexpr uint16_t max = (1 << 2);
}
namespace b {
constexpr uint16_t min = (1 << 3);
constexpr uint16_t mid = (1 << 4);
constexpr uint16_t max = (1 << 5);
}
namespace c {
constexpr uint16_t min = (1 << 6);
constexpr uint16_t mid = (1 << 7);
constexpr uint16_t max = (1 << 8);
}
namespace d {
constexpr uint16_t min = (1 << 9);
constexpr uint16_t mid = (1 << 10);
constexpr uint16_t max = (1 << 11);
}

namespace flight_mode {
constexpr uint16_t angle = a::min;
constexpr uint16_t acro = a::max | b::min;
constexpr uint16_t direct = a::max | b::max;
} // namespace flight_mode
namespace calibration {
constexpr uint16_t accel = a::min | c::mid | d::min;
constexpr uint16_t gyro = a::max | b::min | c::mid | d::min;
} // namespace calibration
namespace cfg {
constexpr uint16_t lights = c::min | d::max;

constexpr uint16_t accel_lpf = a::min | c::max | d::min;
constexpr uint16_t mahony = a::min | c::max | d::min;

constexpr uint16_t imu_lpf = a::max | b::min | c::max | d::min;

constexpr uint16_t angle_p = a::min | c::max | d::min;
constexpr uint16_t angle_i = a::min | c::max | d::min;

constexpr uint16_t rate_p = a::max | b::min | c::max | d::min;
constexpr uint16_t rate_d = a::max | b::min | c::max | d::min;

constexpr uint16_t yaw_rate_p = c::max | d::max;
constexpr uint16_t yaw_rate_i = c::max | d::max;
} // namespace cfg


} // namespace switch_state

#if 0
#define RC_SW_A1 (0x0001 << 0)
#define RC_SW_A2 (0x0001 << 1)
#define RC_SW_A3 (0x0001 << 2)
#define RC_SW_B1 (0x0001 << 3)
#define RC_SW_B2 (0x0001 << 4)
#define RC_SW_B3 (0x0001 << 5)
#define RC_SW_C1 (0x0001 << 6)
#define RC_SW_C2 (0x0001 << 7)
#define RC_SW_C3 (0x0001 << 8)
#define RC_SW_D1 (0x0001 << 9)
#define RC_SW_D2 (0x0001 << 10)
#define RC_SW_D3 (0x0001 << 11)

// Config
#define RC_ST_FM_ANGLE ( (switch_state::min::a) ) // 0000 0001
#define RC_ST_FM_ACRO (RC_SW_A3 + RC_SW_B1) // 0000 0110
#define RC_ST_FM_DIRECT (RC_SW_A3 + RC_SW_B3) // 0000 1010

#define RC_CFG_LIGHTS (RC_SW_C1 + RC_SW_D3) // 1 0001 0000

#define RC_ST_CAL_ACCEL (RC_SW_A1 + RC_SW_C2 + RC_SW_D1) // 1010 0001
// #define RC_CFG_ACCEL_LPF (RC_SW_A1 + RC_SW_C3 + RC_SW_D1) // 1010 0001
// #define RC_CFG_MAHONY (RC_SW_A1 + RC_SW_C3 + RC_SW_D1) // 1010 0001

#define RC_ST_CAL_GYRO (RC_SW_A3 + RC_SW_B1 + RC_SW_C2 + RC_SW_D1) // 1010 0110
// #define RC_CFG_IMU_LPF (RC_SW_A3 + RC_SW_B1 + RC_SW_C3 + RC_SW_D1) // 1010 0110

// #define RC_CFG_ANGLE_P (RC_SW_A1 + RC_SW_C3 + RC_SW_D1) // 1100 0001
// #define RC_CFG_ANGLE_I (RC_SW_A1 + RC_SW_C3 + RC_SW_D1) // 1100 0001

// #define RC_CFG_RATE_P (RC_SW_A3 + RC_SW_B1 + RC_SW_C3 + RC_SW_D1) // 1100 0110
// #define RC_CFG_RATE_D (RC_SW_A3 + RC_SW_B1 + RC_SW_C3 + RC_SW_D1) // 1100 0110

// #define RC_CFG_YAW_P (RC_SW_C3 + RC_SW_D3) // 1 0010 0000
// #define RC_CFG_YAW_I (RC_SW_C3 + RC_SW_D3) // 1 0010 0000
#endif



namespace max_input {
uint8_t constexpr throttle_dc = 125; // duty cycle
uint8_t constexpr angle_tilt_deg = 45; // degrees
uint8_t constexpr rate_tilt_degps = 250; // degrees per second
uint8_t constexpr rate_yaw_degps = 125; // degrees per second
uint8_t constexpr tilt_dc = 13; // duty cycle
uint8_t constexpr yaw_dc = 15; // duty cycle
} // namespace max_input


namespace a_b {
enum class k_Map : int16_t {
  A1B1 = 0,
  A2B1 = 333,
  A1B2 = 666,
  A2B2 = 1000,
  Count,
  Step = 333,
};
} // namespace a_b

namespace c_d {
enum class k_Map : int16_t {
  C1D1 = 0,
  C2D1 = 200,
  C3D1 = 400,
  C1D2 = 600,
  C2D2 = 800,
  C3D2 = 1000,
  Count,
  Step = 200,
};
} // namespace c_d


namespace div {
constexpr uint8_t throttle = (input::max - input::min) / max_input::throttle_dc;
constexpr uint8_t angle_tilt = ((input::max - input::min) / 2) / max_input::angle_tilt_deg;
constexpr uint8_t rate_tilt = ((input::max - input::min) / 2) / max_input::rate_tilt_degps;
constexpr uint8_t rate_yaw = ((input::max - input::min) / 2) / max_input::rate_yaw_degps;
constexpr uint8_t tilt = ((input::max - input::min) / 2) / max_input::tilt_dc;
constexpr uint8_t yaw = ((input::max - input::min) / 2) / max_input::yaw_dc;
constexpr uint16_t sw_ab = (int16_t)a_b::k_Map::Step / 2;
constexpr uint16_t sw_cd = (int16_t)c_d::k_Map::Step / 2;
} // namespace div

int16_t constexpr pot_step = 333;

} // namespace rx
} // namespace cfg
