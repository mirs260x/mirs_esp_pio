#pragma once
#include <Arduino.h>

/** @brief 単一MD10Cチャンネルのデバイス抽象。duty出力のみ。
 *  @details 正逆の意味づけは持たない（ペア層の仕事）。非常停止の判断も持たない（制御層の仕事）。 */
class MotorDriver {
public:
    static constexpr int DUTY_MAX = 255;

    MotorDriver(uint8_t pin_pwm, uint8_t pin_dir);

    void begin(uint32_t pwm_freq = 20000, uint8_t pwm_resolution = 8);
    void setDuty(int duty);  // -255..+255、範囲外はclamp。正=DIR HIGH
    void stop();

private:
    uint8_t pin_pwm_;
    uint8_t pin_dir_;
};
