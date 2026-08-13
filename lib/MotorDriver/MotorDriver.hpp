#pragma once
#include <Arduino.h>

// Cytron MD10C (PWM+DIR方式) モータドライバ制御クラス
class MotorDriver {
public:
    MotorDriver(uint8_t pwm_pin, uint8_t dir_pin);

    void begin();

    // speed: -1.0 (全速後進) 〜 1.0 (全速前進)
    void setSpeed(float speed);

    void stop();

private:
    uint8_t _pwm_pin;
    uint8_t _dir_pin;

    static constexpr int PWM_FREQ = 20000;  // 20kHz (可聴域外)
    static constexpr int PWM_RES  = 8;      // 8bit (0-255)
};
