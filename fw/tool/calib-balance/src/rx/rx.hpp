#pragma once
#include <stdint.h>


namespace rx {

struct State_t {
	uint8_t throttle = 0;
	int16_t pitch = 0;
	int16_t roll = 0;
	int16_t yaw = 0;
};

enum class k_States4 : int16_t {
	SW_11 = 0,
	SW_21 = 333,
	SW_12 = 667,
	SW_22 = 1000,
};

enum class k_States5 : int16_t {
	SW_11 = 0,
	SW_21 = 200,
	SW_31 = 400,
	SW_12 = 600,
	SW_22 = 800,
	SW_32 = 1000,
};

enum class k_States6 : int16_t {
	SW_11 = 0,
	SW_21 = 125,
	SW_31 = 250,
	SW_12 = 375,
	SW_22 = 500,
	SW_32 = 625,
	SW_13 = 750,
	SW_23 = 875,
	SW_33 = 1000,
};


/*
|  | 2+2  | 2+3  | 3+2  | 3+3  |
|--|------|------|------|------|
|11| 1000 | 1000 | 1000 | 1000 |
|21| 1333 | 1600 | 1200 | 1125 |
|31|      |      | 1400 | 1250 |
|12| 1666 | 1200 | 1600 | 1375 |
|22| 2000 | 1800 | 1800 | 1500 |
|32|      |      | 2000 | 1625 |
|13|      | 1400 |      | 1750 |
|23|      | 2000 |      | 1875 |
|33|      |      |      | 2000 |
*/


constexpr uint8_t divider[3] = {
	(333 / 2),
	(200 / 2),
	(125 / 2)
};



void ReadRx(uint8_t ino_pin, int16_t rc_raw[], uint8_t channels_count);

bool SplitChannel(int16_t ch_value,
	int16_t& ch1_value, uint8_t ch1_states,
	int16_t& ch2_value, uint8_t ch2_states);

uint16_t CombineChannels(int16_t const switches[], uint8_t count);

} // namespace rx
