#include "UART_atmega328.hpp"
#include <avr/io.h>
#include <stdlib.h>

namespace m328 {
namespace uart {

bool initialize(const uint32_t baud) {
  // uint16_t ubrr = (f_cpu/(16*baud))-1;

  if (baud == 0) return false;

// #define UBRR_VALUE (((F_CPU) + 8UL * (baud)) / (16UL * (baud)) -1UL)
  uint16_t ubrr_value = (((F_CPU) + 8UL * (baud)) / (16UL * (baud)) - 1UL);

// #define BAUD_TOL 2
// #if 100 * (F_CPU) > (16 * ((UBRR_VALUE) + 1)) * (100 * (baud) + (baud) * (BAUD_TOL))
// #  define USE_2X 1
// #elif 100 * (F_CPU) < (16 * ((UBRR_VALUE) + 1)) * (100 * (baud) - (baud) * (BAUD_TOL))
// #  define USE_2X 1
// #else
// #  define USE_2X 0
// #endif

  bool use_2x = false;
  const uint8_t baud_tol = 2;
  if (100 * (F_CPU) > (16 * ((ubrr_value) + 1)) * (100 * (baud) + (baud) * (baud_tol))) {
    use_2x = true;
  } else if (100 * (F_CPU) < (16 * ((ubrr_value) + 1)) * (100 * (baud) - (baud) * (baud_tol))) {
    use_2x = true;
  }

// #if USE_2X
// #  undef UBRR_VALUE
// #  define UBRR_VALUE (((F_CPU) + 4UL * (baud)) / (8UL * (baud)) -1UL)

  if (use_2x) {
    ubrr_value = (((F_CPU) + 4UL * (baud)) / (8UL * (baud)) - 1UL);

// #  if 100 * (F_CPU) > (8 * ((UBRR_VALUE) + 1)) * (100 * (baud) + (baud) * (BAUD_TOL))
// #    warning "Baud rate achieved is higher than allowed"
//   return false;
// #  endif

    if (100 * (F_CPU) > (8 * ((ubrr_value) + 1)) * (100 * (baud) + (baud) * (baud_tol))) {
// #    warning "Baud rate achieved is higher than allowed"
      return false;
    }

// #  if 100 * (F_CPU) < (8 * ((UBRR_VALUE) + 1)) * (100 * (baud) - (baud) * (BAUD_TOL))
// #    warning "Baud rate achieved is lower than allowed"
//     return false;
// #  endif
// #endif // USE_2X
    if (100 * (F_CPU) < (8 * ((ubrr_value) + 1)) * (100 * (baud) - (baud) * (baud_tol))) {
// #    warning "Baud rate achieved is lower than allowed"
      return false;
    }
  }

// #define UBRRL_VALUE (UBRR_VALUE & 0xff)
// #define UBRRH_VALUE (UBRR_VALUE >> 8)

  uint8_t ubrrl = ubrr_value & 0xff;
  uint8_t ubrrh = ubrr_value >> 8;


  // Set baud rate
  /* This is a 12-bit register which contains the USART baud rate. The UBRRnH contains the four most
  significant bits and the UBRR0L contains the eight least significant bits of the USART baud rate. Ongoing
  transmissions by the Transmitter and Receiver will be corrupted if the baud rate is changed. Writing
  UBRRnL will trigger an immediate update of the baud rate prescaler. */
  // UBRR0H = (uint8_t)(ubrr >> 8);
  // UBRR0L = (uint8_t)ubrr;
  UBRR0H = ubrrh;
  UBRR0L = ubrrl;

  // USART Control and Status Registers
  // A
  // U2Xn doubles the UART transmission speed
// #if USE_2X
//   UCSR0A |= (1 << U2X0);
// #else
//   UCSR0A &= ~(1 << U2X0);
// #endif
  if (use_2x) {
    UCSR0A |= (1 << U2X0);
  } else {
    UCSR0A &= ~(1 << U2X0);
  }

  // B
  // RXENn Enable
  // TXENn Enable
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  // C
  // USBSn USART Stop Bit Select
  // UCSZn USART Character Size
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

  return true;
}

void print(char data) {
  // UDREn USART Data Register Empty
  while ((UCSR0A & (1 << UDRE0)) == 0) {}
  UDR0 = data;
}

void print(const char string[]) {
  uint8_t i = 0;
  while (string[i] != '\0') {
    print(string[i]);
    ++i;
  }
}

uint8_t read() {
  while (!(UCSR0A & (1 << RXC0)));
  return UDR0;
}

void print(int8_t number, uint8_t radix) {
  char c[9];
  itoa(number, c, radix);
  print(c);
}

void print(int16_t number, uint8_t radix) {
  char c[17];
  itoa(number, c, radix);
  print(c);
}

void print(int32_t number, uint8_t radix) {
  char c[33];
  ltoa(number, c, radix);
  print(c);
}

void print(uint8_t number, uint8_t radix) {
  char c[9];
  utoa(number, c, radix);
  print(c);
}

void print(uint16_t number, uint8_t radix) {
  char c[17];
  utoa(number, c, radix);
  print(c);
}

void print(uint32_t number, uint8_t radix) {
  char c[33];
  ultoa(number, c, radix);
  print(c);
}

void print(float number, uint8_t precision) {
  char c[33];
  dtostrf(number, 10, precision, c);
  print(c);
}

} //namespace uart
} //namespace m328


// void printChar(char data) {
//   // UDREn USART Data Register Empty
//   while ((UCSR0A & (1<<UDRE0)) == 0) {}
//   UDR0 = data;
// }

// void printString(const char string[]) {
//   uint8_t i = 0;
//   while (string[i] != '\0') {
//     printChar(string[i]);
//     ++i;
//   }
// }

// void printInteger(uint32_t number) {
//   char c[12];
//   itoa(number, c, 10);
//   printString(c);
// }

// void printFloat(float number) {
//   char c[12];
//   dtostrf(number, 10, 3, c);
//   printString(c);
// }


// uint8_t read() {
//   // RXCn USART Receive Complete
//   while ((UCSR0A & (1<<RXC0)) == 0) {}
//   return UDR0;
// }

// void readString(char myString[], uint8_t maxLength) {
//   char response;
//   uint8_t i;
//   i = 0;
//   while (i < (maxLength - 1)) {                   /* prevent over-runs */
//     response = read();
//     print(response);                                    /* echo */
//     if (response == '\r') {                     /* enter marks the end */
//       break;
//     }
//     else {
//       myString[i] = response;                       /* add in a letter */
//       i++;
//     }
//   }
//   myString[i] = 0;                          /* terminal NULL character */
// }



