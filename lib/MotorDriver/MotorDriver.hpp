#pragma once
#include <Arduino.h>

class MotorDriver {
public:
    MotorDriver(uint8_t pwm_pin, uint8_t dir_pin);
    void begin();
    void setSpeed(float speed);
    void stop();

private:
    uint8_t _pwm_pin;
    uint8_t _dir_pin;
		// MD10C R3 supported upto 20kHz
    static constexpr int PWM_FREQENCY = 20000;
    static constexpr int PWM_RESOLUTION = 8;      // 8bit (0-255)
};
