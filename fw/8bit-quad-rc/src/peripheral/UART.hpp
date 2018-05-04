#pragma once

#include <inttypes.h>

void initUART(uint32_t baud, uint32_t f_cpu);

void print(char data);

void print(const char string[]);

void print(int8_t number);

void print(int16_t number);

void print(int32_t number);

void print(uint8_t number);

void print(uint16_t number);

void print(uint32_t number);

void print(float number);

// void printChar(char data);
// void printString(const char string[]);
// void printUI(uint16_t number);
// void printFloat(float number);

// uint8_t read();
// void readString(char myString[], uint8_t maxLength);