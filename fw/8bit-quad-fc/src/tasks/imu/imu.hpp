#ifndef __IMU_HPP__
#define __IMU_HPP__

namespace imu {

static void Init() __attribute__((always_inline));

// -----

static inline void Init() {

}

bool GetAccel(int16_t *x, int16_t *y, int16_t  *z);
bool GetGyro(int16_t *x, int16_t *y, int16_t  *z);
bool getAccelAndGyro(int16_t* ax, int16_t* yx, int16_t* zx,
                     int16_t* gx, int16_t* gy, int16_t* gz);

} // namespace imu

#endif // __IMU_HPP__

#pragma once

#include <avr/power.h>

namespace portable {
namespace imu {



static void Init() __attribute__((always_inline));

static inline void Init() {
	clock_prescale_set(clock_div_8);
}

} // namespace imu
} // namepsace portable
