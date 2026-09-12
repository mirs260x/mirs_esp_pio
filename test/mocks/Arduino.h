// Arduino API stub declarations for native host tests (`pio test -e native`).
// Definitions live in test/test_encoder/test_main.cpp (single TU).
#pragma once
#include <cstdint>

#define HIGH 1
#define LOW 0
#define INPUT_PULLUP 2
#define RISING 3
#define CHANGE 4
#define IRAM_ATTR

void pinMode(uint8_t pin, uint8_t mode);
int digitalRead(uint8_t pin);
int digitalPinToInterrupt(uint8_t pin);
void attachInterruptArg(uint8_t pin, void (*handler)(void *), void *arg, int mode);
void noInterrupts();
void interrupts();
