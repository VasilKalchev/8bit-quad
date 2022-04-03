#pragma once


namespace cfg {

constexpr uint32_t baud_rate = 2000000;

namespace imu {
constexpr uint32_t clock_speed = 800000;
constexpr uint8_t sample_rate_divider = 0;

/*
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
*/
constexpr uint8_t dlpf = 0;

enum class k_AccelRange : uint8_t {
	_2g = 0,
	_4g = 1,
	_8g = 2,
	_16g = 3,
};

enum class k_GyroRange : uint8_t {
	_250dps = 0,
	_500dps = 1,
	_1000dps = 2,
	_2000dps = 3,
};

constexpr k_AccelRange accel_range = k_AccelRange::_2g;
constexpr k_GyroRange gyro_range = k_GyroRange::_250dps;

namespace offset {
namespace accel {
int16_t x = -5362;
int16_t y = -6011;
int16_t z = 1421;
} // namespace accel
namespace gyro {
int16_t x = 51;
int16_t y = -62;
int16_t z = 11;
} // namespace gyro
} // namespace offset
} // namespace imu

} // namespace cfg
