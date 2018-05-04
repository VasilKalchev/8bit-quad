#pragma once
#include <inttypes.h>
#include <math.h>


extern const float toDegrees;
extern const uint8_t expMap[];

int16_t clamp(int16_t value, int16_t minimum, int16_t maximum);

void swap(float& a, float& b);

float lowPassFilter(float current, float previous, float alpha);

bool compareFloat(float x, float y, float epsilon = 0.005f);

template <typename T> int sign(T val) {
	return (T(0) < val) - (val < T(0));
}

enum State : uint8_t {
	ON = 1, OFF = 0, FULL_ON = 255, HALF_ON = 127,
};

enum class Status {
	normal, warning, error,
};

float calculateAltitude(float pressure, float relativeToPressure,
                        float temperature=0);