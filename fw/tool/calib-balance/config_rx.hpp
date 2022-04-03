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


namespace mode {
constexpr uint16_t off = a::min | b::min | c::min | d::min;

constexpr uint16_t calib_accel = a::max | b::min | c::mid | d::min;
constexpr uint16_t calib_gyro = a::max | b::min | c::mid | d::max;

constexpr uint16_t rms_accel = a::max | b::max | c::min | d::min;

constexpr uint16_t accel = a::max | b::max | c::mid | d::min;
constexpr uint16_t gyro = a::max | b::max | c::mid | d::max;
} // namespace mode

} // namespace switch_state


namespace max_input {
uint8_t constexpr throttle_dc = 125; // duty cycle
uint8_t constexpr angle_tilt_deg = 45; // degrees
uint8_t constexpr rate_tilt_degps = 166; // degrees per second
uint8_t constexpr rate_yaw_degps = 125; // degrees per second
uint8_t constexpr tilt_dc = 18; // duty cycle
uint8_t constexpr yaw_dc = 22; // duty cycle
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
