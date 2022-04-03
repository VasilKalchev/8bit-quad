#pragma once
#include <stdint.h>
#include <math.h>


namespace util {

extern const float toDegrees;
extern const uint8_t expMap[];

int16_t clamp(int16_t value, int16_t minimum, int16_t maximum);

void swap(float& a, float& b);

float lowPassFilter(float current, float previous, float alpha);

bool compareFloat(float x, float y, float epsilon = 0.005f);

template <typename T> int sign(T val) {
	return (T(0) < val) - (val < T(0));
}

float calculateAltitude(float pressure, float relativeToPressure,
                        float temperature=0);

int16_t Median(int16_t a, int16_t b, int16_t c);
float Median(float a, float b, float c);

} // namespace util
