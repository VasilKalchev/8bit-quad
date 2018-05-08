#pragma once
#include <UART_atmega328.hpp>

#if DEBUGGING
# if VERBOSE
#   warning "MAD-FC-2a: Debugging is enabled."
#   define DEBUG_INITIALIZE(baud) uart::initialize(baud);
#   define DEBUG(...) uart::print(__LINE__); uart::print(': '); uart::print(__PRETTY_FUNCTION__); uart::print(': '); uart::print(__VA_ARGS__);
#   define DEBUGLN(...)  uart::print(__LINE__); uart::print(': '); uart::print(__PRETTY_FUNCTION__); uart::print(': '); uart::print(__VA_ARGS__); uart::print("\n");
# else // no verbose
#   define DEBUG_INITIALIZE(baud) uart::initialize(baud);
#   define DEBUG(...) uart::print(__VA_ARGS__);
#   define DEBUGLN(...) uart::print(__VA_ARGS__); uart::print("\n");
# endif // ~VERBOSE
#else // no debugging
# define DEBUG_INITIALIZE(baud)
# define DEBUG(...)
# define DEBUGLN(...)
#endif // ~DEBUGGING