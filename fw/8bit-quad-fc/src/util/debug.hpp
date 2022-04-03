#pragma once


#ifndef DEBUG
  #define DEBUG false
#endif

// #define UART_DEBUGGING true

// Cycle periods
#define PRINT_LOOP_PERIOD false
#define DEBUG_LOOP_PERIOD false

#define PRINT_RATE_CTRL_PERIOD false
#define DEBUG_RATE_CTRL_PERIOD false

#define PRINT_ANGLE_CTRL_PERIOD false
#define DEBUG_ANGLE_CTRL_PERIOD false

#define PRINT_FUSION_PERIOD false
#define DEBUG_FUSION_PERIOD false

#define PRINT_QUAT2EULER_PERIOD false
#define DEBUG_QUAT2EULER_PERIOD false
// ~ Cycle periods

// Controllers
#define PRINT_CTRL_PITCH false
#define PRINT_CTRL_ROLL false
#define PRINT_CTRL_YAW false
// ~ Controllers

// IMU
#define PRINT_ANGULAR_RATE false
#define PRINT_ACCELERATION false
// ~ IMU

// Fusion
#define PRINT_ATTITUDE false
// ~ Fusion

#define PRINT_THROTTLE false
#define PRINT_MOTOR_SIGNALS false

#define PRINT_BATTERY_VOLTAGE false

// RC
#define PRINT_RC_RAW false
#define PRINT_RC_SWITCHES_STATE false
#define PRINT_RC_POTENTIOMETERS false
#define PRINT_RC_SWITCHES_MASK false
#define PRINT_RC false
// ~ RC


#if DEBUG
  #warning "MAD 0 RC: General debugging is enabled."
  #define UART_INIT(x) Serial.begin(x)
  #define PRINT(x) Serial.print(x)
#else
  #define UART_INIT(x)
  #define PRINT(x)
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

#if DEBUG_VERBOSE == true
  #include <WProgram.h>
  #define PRINT_VERBOSE(str)    \
    Serial.print(millis());     \
    Serial.print(": ");    \
    Serial.print(__PRETTY_FUNCTION__); \
    Serial.print(' ');      \
    Serial.print(__FILE__);     \
    Serial.print(':');      \
    Serial.print(__LINE__);     \
    Serial.print(' ');      \
    Serial.println(str);
#else
  #define PRINT_VERBOSE(str)
#endif


#if DEBUG_TIMING
  #define TIME_START(NAME) uint32_t time_start_##NAME = micros()

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
