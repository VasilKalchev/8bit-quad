#pragma once

#include "../config/config.hpp"


#if DEBUGGING
#warning "Remote: Debugging is enabled."
#define INIT_UART(x) Serial.begin(x);
#define DEBUG(...) Serial.print(__VA_ARGS__);
#define DEBUGLN(...) Serial.println(__VA_ARGS__);
#else
#define INIT_UART(x)
#define DEBUG(...)
#define DEBUGLN(...)
#endif
