#ifndef PORTABLE_IMU_INCLUDED
#define PORTABLE_IMU_INCLUDED

#include "MPU925x.hpp"

static MPU925x mpu925x;


namespace portable {
namespace imu {

static void Init() __attribute__((always_inline));
static void Read() __attribute__((always_inline));

// --------------------------------------------------------

static inline void Init() {

}

static inline void Read() {

}


} // namespace imu
} // namespace portable


#endif // PORTABLE_IMU_INCLUDED