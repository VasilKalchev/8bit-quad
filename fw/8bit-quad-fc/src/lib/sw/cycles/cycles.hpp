#ifndef __IMU_HPP__
#define __IMU_HPP__

int8_t GetBufferIndex(int8_t target_cycle, int8_t current_cycle,
					  int8_t cycles_count);

#endif // __IMU_HPP__