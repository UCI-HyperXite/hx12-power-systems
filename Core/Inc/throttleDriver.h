#ifndef THROTTLE
#define THROTTLE


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "stm32h7xx_hal.h"

#define DAC_MAX_VALUE 4095 //granularity = 12 bits, 4096 increments (1.22mV per change)
#define DAC_ADDR 0xC6 // throttle DAC address (0x63 shifted left, 0x63 << 1 = 0xC6)
#define STEP_DELAY_MS 5 // specifies HAL delay for accelerating and decelerating 

extern DAC_HandleTypeDef hdac1; //defined in main
extern I2C_HandleTypeDef hi2c1;


void DAC_SetValue(uint16_t val);

void throttleTest();

void accelerate(int targetVal);

void decelerate(int targetVal);

void killThrottle();

#endif
