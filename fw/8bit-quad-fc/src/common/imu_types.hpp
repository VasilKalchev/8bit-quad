#pragma once
#include <inttypes.h>

struct AngularVelocity {
	float y = 0.0f;
	float x = 0.0f;
	float z = 0.0f;
};

struct Acceleration {
	int16_t y = 0;
	int16_t x = 0;
	int16_t z = 0;
};

struct Attitude {
	float pitch = 0.0f;
	float roll = 0.0f;
};