#include "MAD-Utils.hpp"

namespace mad {
namespace util {

int16_t clamp(const int16_t value, const int16_t minimum, const int16_t maximum) {
  return (value < minimum) ? minimum : (value > maximum) ? maximum : value;
}

constexpr float radiansToDegrees(const float radians) {
  return radians * 57.2957795131f;
}

void swap(float& a, float& b) {
  a = a + b;
  b = a - b;
  a = a - b;
}

void lowPassFilter(float &current, float &previous, const float alpha) {
  // return ((current * alpha) + (previous * (1.0f - alpha)));
  current = (current * alpha) + (previous * (1.0f - alpha));
  previous = current;
}

bool compareFloat(const float x, const float y, const float epsilon) {
  return (fabs(x - y) < epsilon);
}

constexpr uint32_t msToS(const uint32_t ms) { return ms / 1000; }
constexpr uint32_t usToS(const uint32_t us) { return us / 1000000; }

// Alternate method to calculate arctangent from: http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm
float arctan2(const float y, const float x) {
  float coeff_1 = PI / 4;
  float coeff_2 = 3 * coeff_1;
  float abs_y = fabs(y) + 1e-10;    // kludge to prevent 0/0 condition
  float r, angle;

  if (x >= 0) {
    r = (x - abs_y) / (x + abs_y);
    angle = coeff_1 - coeff_1 * r;
  } else {
    r = (x + abs_y) / (abs_y - x);
    angle = coeff_2 - coeff_1 * r;
  }
  if (y < 0) {
    return (-angle);    // negate if in quad III or IV
  } else {
    return (angle);
  }
}


MedianFilter::MedianFilter() {
  for (int index = 0; index < DATASIZE; index++) {
    data[index] = 0;
    sortData[index] = 0;
  }
  dataIndex = 0;
}


const float MedianFilter::filter(float newData)
{
  int temp, j; // used to sort array

  // Insert new data into raw data array round robin style
  data[dataIndex] = newData;
  if (dataIndex < (DATASIZE - 1))
  {
    dataIndex++;
  }
  else
  {
    dataIndex = 0;
  }

  // Copy raw data to sort data array
  memcpy(sortData, data, sizeof(data));

  // Insertion Sort
  for (int i = 1; i <= (DATASIZE - 1); i++) {
    temp = sortData[i];
    j = i - 1;
    while (temp < sortData[j] && j >= 0) {
      sortData[j + 1] = sortData[j];
      j = j - 1;
    }
    sortData[j + 1] = temp;
  }
  return data[(DATASIZE) >> 1]; // return data value in middle of sorted array
}

} //namespace util
} //namespace mad