#include "cycles.hpp"

namespace cycles {

int8_t GetBufferIndex(int8_t target_cycle, int8_t current_cycle,
					  int8_t cycles_count) {
	int8_t buffer_index = cycle + (cycles_count - target_cycle);
	if (buffer_index >= cycles_count) {
	  buffer_index -= cycles_count;
	}

	return (buffer_index);
}

} // namespace cycles