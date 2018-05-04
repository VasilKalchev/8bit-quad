/*
 * UART_atmega328 library - HelloUART.ino
 *
 * Created May 4, 2018
 * by Vasil Kalchev
 *
 * https://github.com/VasilKalchev/UART_atmega328
 *
 */

#include <UART_atmega328.hpp>

/* This library uses namespaces for encapsulation. ATmega328 specific
   code is under the namespace 'm328', the UART code is under the
   namespace 'uart'. */
using namespace m328; // namespace for ATmega328 specific code.

void setup() {
  uart::initialize(250000);
}

void loop() {
  uart::print(250000); uart::print("\n"); // Integer in decimal.
  uart::print(250000, 16); uart::print("\n"); // Integer in hexadecimal.
  uart::print(250000.123f, 4); uart::print("\n"); // Float.
  uart::print("Hello UART\n"); // String.
}