#include "utils.hpp"


const float toDegrees = 57.29577951308232087679815481410517033240547246656432154916;
const uint8_t expMap[] = {0, 1, 2, 3, 5, 9, 15, 24, 39, 63, 101, 160, 255};

int16_t clamp(int16_t value, int16_t minimum, int16_t maximum) {
	return (value < minimum) ? minimum : (value > maximum) ? maximum : value;
}

float clamp(float value, float minimum, float maximum) {
	if (value > maximum) return maximum;
	else if (value < minimum) return minimum;
	else return value;
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
