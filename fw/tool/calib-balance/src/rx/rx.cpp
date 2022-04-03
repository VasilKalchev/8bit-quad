#include "rx.hpp"
#include "../../config_rx.hpp"
#include "PPMReader/PPMReader.h"


namespace rx {

void ReadRx(uint8_t ino_pin, int16_t rc_raw[], uint8_t channels_count) {
	static PPMReader ppm(ino_pin, channels_count);

	rc_raw[(uint8_t)cfg::rx::k_Ch::Roll] = ppm.rawChannelValue(
      (uint8_t)cfg::rx::k_Ch::RHoriz+1);
    rc_raw[(uint8_t)cfg::rx::k_Ch::Pitch] = ppm.rawChannelValue(
      (uint8_t)cfg::rx::k_Ch::RVert+1);
    rc_raw[(uint8_t)cfg::rx::k_Ch::Throttle] = ppm.rawChannelValue(
      (uint8_t)cfg::rx::k_Ch::LVert+1);
    rc_raw[(uint8_t)cfg::rx::k_Ch::Yaw] = ppm.rawChannelValue(
      (uint8_t)cfg::rx::k_Ch::LHoriz+1);

    #if RC_SW_A
    rc_raw[(uint8_t)cfg::rx::k_Ch::SwA] = ppm.rawChannelValue(
      (uint8_t)cfg::rx::k_Ch::SwA+1);
    #endif
    #if RC_SW_B
    rc_raw[(uint8_t)cfg::rx::k_Ch::SwB] = ppm.rawChannelValue(
      (uint8_t)cfg::rx::k_Ch::SwB+1);
    #endif
    #if RC_SW_C
    rc_raw[(uint8_t)cfg::rx::k_Ch::SwC] = ppm.rawChannelValue(
      (uint8_t)cfg::rx::k_Ch::SwC+1);
    #endif
    #if RC_SW_D
    rc_raw[(uint8_t)cfg::rx::k_Ch::SwD] = ppm.rawChannelValue(
      (uint8_t)cfg::rx::k_Ch::SwD+1);
    #endif
    #if RC_SW_AB
    rc_raw[(uint8_t)cfg::rx::k_Ch::SwAB] = ppm.rawChannelValue(
      (uint8_t)cfg::rx::k_Ch::SwAB+1);
    #endif
    #if RC_SW_CD
    rc_raw[(uint8_t)cfg::rx::k_Ch::SwCD] = ppm.rawChannelValue(
      (uint8_t)cfg::rx::k_Ch::SwCD+1);
    #endif
    #if RC_VR_A
    rc_raw[(uint8_t)cfg::rx::k_Ch::VrA] = ppm.rawChannelValue(
      (uint8_t)cfg::rx::k_Ch::VrA+1);
    #endif
    #if RC_VR_B
    rc_raw[(uint8_t)cfg::rx::k_Ch::VrB] = ppm.rawChannelValue(
      (uint8_t)cfg::rx::k_Ch::VrB+1);
    #endif
    #if RC_SNR
    rc_raw[(uint8_t)cfg::rx::k_Ch::SNR] = ppm.rawChannelValue(
      (uint8_t)cfg::rx::k_Ch::SNR+1);
    #endif
    #if RC_ERROR
    rc_raw[(uint8_t)cfg::rx::k_Ch::ERROR] = ppm.rawChannelValue(
      (uint8_t)cfg::rx::k_Ch::ERROR+1);
    #endif
    #if RC_PPM1
    rc_raw[(uint8_t)cfg::rx::k_Ch::PPM1] = ppm.rawChannelValue(
      (uint8_t)cfg::rx::k_Ch::PPM1+1);
    #endif
    #if RC_PPM2
    rc_raw[(uint8_t)cfg::rx::k_Ch::PPM2] = ppm.rawChannelValue(
      (uint8_t)cfg::rx::k_Ch::PPM2+1);
    #endif
    #if RC_PPM3
    rc_raw[(uint8_t)cfg::rx::k_Ch::PPM3] = ppm.rawChannelValue(
      (uint8_t)cfg::rx::k_Ch::PPM3+1);
    #endif
}

bool SplitChannel(int16_t ch_value,
	int16_t& ch1_value, uint8_t ch1_states,
	int16_t& ch2_value, uint8_t ch2_states) {

	if (ch1_states < 2 || ch1_states > 3
		|| ch2_states < 2 || ch2_states > 3) {
		return false;
	}

	bool ret = true;

	uint8_t states_total = ch1_states + ch2_states;

	uint8_t value_code = (
		( (ch_value - cfg::rx::input::min) / divider[states_total-4] ) + 1) / 2;

	if ((ch1_states == 2) & (ch2_states == 2)) {
		switch (value_code) {
			case 0: // 1000
			ch1_value = 1000;
			ch2_value = 1000;
			break;
			case 1: // 1333
			ch1_value = 2000;
			ch2_value = 1000;
			break;
			case 2: // 1666
			ch1_value = 1000;
			ch2_value = 2000;
			break;
			case 3: // 2000
			ch1_value = 2000;
			ch2_value = 2000;
			break;
			default:
			ret = false;
			break;
		}
	} else if ((ch1_states == 2) & (ch2_states == 3)) {
		switch (value_code) {
			case 0: // 1000
			ch1_value = 1000;
			ch2_value = 1000;
			break;
			case 1: // 1200
			ch1_value = 1000;
			ch2_value = 1500;
			break;
			case 2: // 1400
			ch1_value = 1000;
			ch2_value = 2000;
			break;
			case 3: // 1600
			ch1_value = 2000;
			ch2_value = 1000;
			break;
			case 4: // 1800
			ch1_value = 2000;
			ch2_value = 1500;
			break;
			case 5: // 2000
			ch1_value = 2000;
			ch2_value = 2000;
			break;
			default:
			ret = false;
			break;
		}
	} else if ((ch1_states == 3) & (ch2_states == 2)) {
		switch (value_code) {
			case 0: // 1000
			ch1_value = 1000;
			ch2_value = 1000;
			break;
			case 1: // 1200
			ch1_value = 1500;
			ch2_value = 1000;
			break;
			case 2: // 1400
			ch1_value = 2000;
			ch2_value = 1000;
			break;
			case 3: // 1600
			ch1_value = 1000;
			ch2_value = 2000;
			break;
			case 4: // 1800
			ch1_value = 1500;
			ch2_value = 2000;
			break;
			case 5: // 2000
			ch1_value = 2000;
			ch2_value = 2000;
			break;
			default:
			ret = false;
			break;
		}
	} else if ((ch1_states == 3) & (ch2_states == 3)) {
		switch (value_code) {
			case 0:
			ch1_value = 1000;
			ch2_value = 1000;
			break;
			case 1:
			ch1_value = 1500;
			ch2_value = 1000;
			break;
			case 2:
			ch1_value = 2000;
			ch2_value = 1000;
			break;
			case 3:
			ch1_value = 1000;
			ch2_value = 1500;
			break;
			case 4:
			ch1_value = 1500;
			ch2_value = 1500;
			break;
			case 5:
			ch1_value = 2000;
			ch2_value = 1500;
			break;
			case 6:
			ch1_value = 1000;
			ch2_value = 2000;
			break;
			case 7:
			ch1_value = 1500;
			ch2_value = 2000;
			break;
			case 8:
			ch1_value = 2000;
			ch2_value = 2000;
			break;
			default:
			ret = false;
			break;
		}
	}

	return ret;
}

uint16_t CombineChannels(int16_t const switches[], uint8_t count) {
	uint16_t mask = 0x0000;

	for (uint8_t i = 0; i < count; ++i) {
		uint8_t sh = (switches[i] / 500) - 2;
		mask |= (1 << (sh + (i * 3) ));
	}

	return mask;
}

} // namespace rx
