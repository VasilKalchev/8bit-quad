#pragma once

#include <inttypes.h>

void initADC();
void initFreerunningADC();

int16_t readADC(uint8_t channel);
int16_t readAnalogAverage(uint8_t pin, uint16_t readings, uint16_t us = 500);