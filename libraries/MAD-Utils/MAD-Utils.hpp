#pragma once
#include <stdint.h>
#include <math.h>
#include <avr/pgmspace.h>

namespace mad {
namespace util {

// const uint8_t expMap[] = {0, 1, 2, 3, 5, 9, 15, 24, 39, 63, 101, 160, 254};
const uint8_t exp[] PROGMEM = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 22, 25, 28, 32, 36, 41, 46, 52, 59, 67, 75, 85, 96, 109, 123, 139, 156, 177, 199, 225, 255};

int16_t clamp(const int16_t value, const int16_t minimum, const int16_t maximum);

constexpr float radiansToDegrees(const float radians);

void swap(float& a, float& b);

void lowPassFilter(float &current, float &previous, const float alpha);

const bool compareFloat(const float x, const float y, const float epsilon = 0.005f);

constexpr uint32_t msToS(const uint32_t ms);
constexpr uint32_t usToS(const uint32_t us);

template <typename T> int8_t sign(const T val) {
	return (T(0) < val) - (val < T(0));
}

float arctan2(const float y, const float x);

#define G_2_MPS2(g) ((g) * 9.80665)
#define MPS2_2_G(m) ((m) * 0.10197162)


class MedianFilter {
public:
	float data[DATASIZE], sortData[DATASIZE];
	int dataIndex;
	MedianFilter();

	void initialize();

	const float filter(float newData);
};

} //namespace util
} //namespace mad


// static float ms2_to_g(const float accel) const;
// static bool ms2_to_g(float &x, float &y, float &z) const;
// static bool ms2_to_g(float (&accel)[3]) const;
// static float g_to_ms2(const float accel) const;
// static bool g_to_ms2(float &x, float &y, float &z) const;
// static bool g_to_ms2(float (&accel)[3]) const;

// static float deg_to_rad(const float gyro) const;
// static bool deg_to_rad(float &x, float &y, float &z) const;
// static bool deg_to_rad(float (&gyro)[3]) const;
// static float rad_to_deg(const float gyro) const;
// static bool rad_to_deg(float &x, float &y, float &z) const;
// static bool rad_to_deg(float (&gyro)[3]) const;

// static float c_to_f(const float c) const;
// static float f_to_c(const float f) const;