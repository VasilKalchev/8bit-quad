*UART_atmega328 is a collection of functions for using the UART on ATmega328.*

[![release](https://img.shields.io/badge/release-0.1.0-yellow.svg)](https://github.com/VaSe7u/UART_atmega328/releases)
[![license](https://img.shields.io/github/license/mashape/apistatus.svg?maxAge=2592000)](https://opensource.org/licenses/mit-license.php)


**UART_atmega328**.
UART_atmega328 library is an alternative to the Arduino's library for working with the UART. It is specifically for the ATmega328, it lacks a lot of functionality and is meant to be used for simple debugging messages while eliminating the requirement of using the Arduino's IDE.


Resources
=========
 - [Examples][examples]
 - [Latest release][latest release]


Features
========
 - Lighter.


Requirements
============
 - ATmega328.


Quick start
===========
```c++
#include <UART_atmega328.hpp>
using namespace m328; // namespace for ATmega328 specific code.

void setup() {
  uart::initialize(250000);
}

void loop() {
  uart::print(250000); uart::print("\n"); // Integer in decimal.
  uart::print(250000, 16); uart::print("\n"); // Integer in hexadecimal.
  uart::print(250000.123f, 4); uart::print("\n"); // Float.
  uart::print("UART print\n"); // String.
}

```

License
=======
The MIT License (MIT)

Copyright (c) 2016 Vasil Kalchev

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

[examples]: https://github.com/VaSe7u/UART_atmega328/tree/master/examples
[latest release]: https://github.com/VaSe7u/UART_atmega328/releases/latest
