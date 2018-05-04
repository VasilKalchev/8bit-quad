#pragma once
#include <Arduino.h>

#include "../config/config.hpp"


#if DEBUGGING
#warning "FC: General debugging is enabled."
#define INIT_UART(x) uart::initialize(x);
#define DEBUG(...) uart::print(__VA_ARGS__);
#define DEBUGLN(...) uart::print(__VA_ARGS__); uart::print("\n");
#else
#define INIT_UART(x)
#define DEBUG(...)
#define DEBUGLN(...)
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