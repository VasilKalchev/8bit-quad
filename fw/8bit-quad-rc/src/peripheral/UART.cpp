#include "UART.hpp"
#include <avr/io.h>
#include <stdlib.h>


void initUART(uint32_t baud, uint32_t f_cpu) {
  uint16_t ubrr = (f_cpu/(16*baud))-1;

  // Set baud rate
  /* This is a 12-bit register which contains the USART baud rate. The UBRRnH contains the four most
  significant bits and the UBRR0L contains the eight least significant bits of the USART baud rate. Ongoing
  transmissions by the Transmitter and Receiver will be corrupted if the baud rate is changed. Writing
  UBRRnL will trigger an immediate update of the baud rate prescaler. */
  UBRR0H = (uint8_t)(ubrr >> 8);
  UBRR0L = (uint8_t)ubrr;

  // USART Control and Status Registers
  // A
  // U2Xn doubles the UART transmission speed
#if USE_2X
  UCSR0A |= (1 << U2X0);
#else
  UCSR0A &= ~(1 << U2X0);
#endif

  // B
  // RXENn Enable
  // TXENn Enable
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  // C
  // USBSn USART Stop Bit Select
  // UCSZn USART Character Size
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
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

void print(int8_t number) {
  char c[9];
  itoa(number, c, 10);
  print(c);
}

void print(int16_t number) {
  char c[17];
  itoa(number, c, 10);
  print(c);
}

void print(int32_t number) {
  char c[33];
  itoa(number, c, 10);
  print(c);
}

void print(uint8_t number) {
  char c[9];
  itoa(number, c, 10);
  print(c);
}

void print(uint16_t number) {
  char c[17];
  itoa(number, c, 10);
  print(c);
}

void print(uint32_t number) {
  char c[33];
  itoa(number, c, 10);
  print(c);
}

void print(float number) {
  char c[33];
  dtostrf(number, 10, 3, c);
  print(c);
}

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