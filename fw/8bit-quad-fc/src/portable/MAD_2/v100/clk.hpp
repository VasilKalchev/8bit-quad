#pragma once

#include <avr/power.h>

namespace portable {
namespace clk {



static void Init() __attribute__((always_inline));

static inline void Init() {
	clock_prescale_set(clock_div_1);
}

} // namespace clk
} // namepsace portable
