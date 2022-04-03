#pragma once
#include <stdint.h>

#include <avr/io.h>


namespace board {

namespace pin {

namespace motor {
constexpr uint8_t tl = PD5;
constexpr uint8_t tr = PB2;
constexpr uint8_t bl = PD6;
constexpr uint8_t br = PB1;
} //namespace motor

namespace indication {
constexpr uint8_t arms = PD3;
constexpr uint8_t lamp = PD4;
constexpr uint8_t signal = PC2;
constexpr uint8_t warning = PC1;
} //namespace indication

namespace input {
namespace ino {
// constexpr uint8_t batteryVoltage = A3;
} //namespace ino
constexpr uint8_t batteryVoltage = PC3;
} //namespace input

} //namespace pin


void Init();

void SetTL(uint8_t pwm);
void SetTR(uint8_t pwm);
void SetBL(uint8_t pwm);
void SetBR(uint8_t pwm);

} // namespace board
