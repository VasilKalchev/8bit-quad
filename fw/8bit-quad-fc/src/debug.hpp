#pragma once
#include <Arduino.h>

#include "../config.hpp"

#if DEBUG_TIMING
    #define TIME_START(NAME) uint32_t time_start_##NAME = micros();

    #define TIME_STOP(NAME, length) static uint16_t time_ndx_##NAME; static uint16_t time_log_##NAME##_s[length]; \
        if (time_ndx_##NAME < length) { \
            time_log_##NAME##_s[time_ndx_##NAME] = micros() - time_start_##NAME; ++time_ndx_##NAME; \
        } else { \
            for (uint16_t i = 0; i < length; ++i) { \
                PRINT(time_log_##NAME##_s[i]); PRINT(", "); \
            } \
            PRINT("\n"); time_ndx_##NAME = 0; \
        }
#else
    #define TIME_START(NAME)
    #define TIME_STOP(NAME, length)
#endif

#if DEBUGGING == true
    #warning "FC: General debugging is enabled."
    #define INIT_UART(x) uart::initialize(x);
    #define PRINT(x) uart::print(x);
    #define DEBUG(...) uart::print(__VA_ARGS__);
    #define DEBUGLN(...) uart::print(__VA_ARGS__); uart::print("\n");
#else
    #define INIT_UART(x)
    #define PRINT(x)
    #define DEBUG(...)
    #define DEBUGLN(...)
#endif


#if DEBUG_PIN == true
    #define DBG_PIN_INIT() DDRD |= (1 << pin::indication::lamp)
    #define DBG_PIN_HIGH() PORTD |= (1 << pin::indication::lamp)
    #define DBG_PIN_LOW() PORTD &= ~(1 << pin::indication::lamp)
    #define DBG_PIN_TOGGLE() PORTD ^= (1 << pin::indication::lamp)
#else
    #define DBG_PIN_INIT()
    #define DBG_PIN_HIGH()
    #define DBG_PIN_LOW()
    #define DBG_PIN_TOGGLE()
#endif

// #if DEBUGGING_COMM
// #warning "FC: Communication debugging is enabled."
// #define DEBUG_COMM(...) Serial.print(__VA_ARGS__);
// #define DEBUGLN_COMM(...) Serial.println(__VA_ARGS__);
// #else
// #define DEBUG_COMM(...)
// #define DEBUGLN_COMM(...)
// #endif

// #if DEBUGGING_IMU
// #warning "FC: IMU debugging is enabled."
// #define DEBUG_IMU(...) Serial.print(__VA_ARGS__);
// #define DEBUGLN_IMU(...) Serial.println(__VA_ARGS__);
// #else
// #define DEBUG_IMU(...)
// #define DEBUGLN_IMU(...)
// #endif

#ifdef DEBUG_V
  #include <WProgram.h>
  #define DEBUG_PRINT(str)    \
    Serial.print(millis());     \
    Serial.print(": ");    \
    Serial.print(__PRETTY_FUNCTION__); \
    Serial.print(' ');      \
    Serial.print(__FILE__);     \
    Serial.print(':');      \
    Serial.print(__LINE__);     \
    Serial.print(' ');      \
    Serial.println(str);
#endif
