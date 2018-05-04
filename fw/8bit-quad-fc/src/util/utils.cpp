#include "utils.hpp"


const float toDegrees = 57.29577951308232087679815481410517033240547246656432154916;
const uint8_t expMap[] = {0, 1, 2, 3, 5, 9, 15, 24, 39, 63, 101, 160, 254};

int16_t clamp(int16_t value, int16_t minimum, int16_t maximum) {
	return (value < minimum) ? minimum : (value > maximum) ? maximum : value;
}

void swap(float& a, float& b) {
	a = a + b;
	b = a - b;
	a = a - b;
}

float lowPassFilter(float current, float previous, float alpha) {
	return ((current * alpha) + (previous * (1 - alpha)));
}

bool compareFloat(float x, float y, float epsilon) {
	return (fabs(x - y) < epsilon);
}

float calculateAltitude(float pressure, float relativeToPressure,
                        float temperature) {
	return ((pow((relativeToPressure / pressure), 0.190284) - 1) * (temperature + 273.15)) / 0.0065;
	// return 44330 * (1.0 - pow(pressure / relativeToPressure, 0.1903));
}