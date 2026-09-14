// Arduino API stub declarations for native host tests (`pio test -e native`).
// Definitions live in each test_*/test_main.cpp (single TU per test).
#pragma once
#include <cstdint>
#include <cstdlib>

#define HIGH 1
#define LOW 0
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 6
#define OUTPUT 5
#define RISING 3
#define CHANGE 4
#define IRAM_ATTR

#ifndef PI
#define PI 3.14159265358979323846
#endif

// constrainは実機Arduinoのマクロ相当。ヘッダinlineで提供する
template <typename T>
inline T constrain(T v, T lo, T hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void pinMode(uint8_t pin, uint8_t mode);
int digitalRead(uint8_t pin);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalPinToInterrupt(uint8_t pin);
void attachInterruptArg(uint8_t pin, void (*handler)(void *), void *arg, int mode);
bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t resolution);
void ledcWrite(uint8_t pin, uint32_t duty);
void noInterrupts();
void interrupts();
unsigned long micros();
unsigned long millis();
