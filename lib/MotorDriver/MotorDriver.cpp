#include "MotorDriver.hpp"

// 外部(main.cpp)で定義されるE-Stopフラグ
// 通信経路と独立して安全側に倒すため、割り込み経由で直接ここを参照する
extern volatile bool g_estop_active;

MotorDriver::MotorDriver(uint8_t pwm_pin, uint8_t dir_pin)
    : _pwm_pin(pwm_pin), _dir_pin(dir_pin) {}

void MotorDriver::begin() {
    pinMode(_dir_pin, OUTPUT);

    // NOTE: Arduino ESP32 core 3.x系のLEDC API (pin単位でattach/write)
    // core 2.x系を使う場合は ledcSetup(ch, freq, res) + ledcAttachPin(pin, ch) + ledcWrite(ch, duty) の形に読み替えること
    ledcAttach(_pwm_pin, PWM_FREQ, PWM_RES);

    stop();
}

void MotorDriver::setSpeed(float speed) {
    if (g_estop_active) {
        stop();
        return;
    }

    speed = constrain(speed, -1.0f, 1.0f);

    digitalWrite(_dir_pin, speed >= 0 ? HIGH : LOW);

    uint8_t duty = (uint8_t)(fabs(speed) * 255.0f);
    ledcWrite(_pwm_pin, duty);
}

void MotorDriver::stop() {
    ledcWrite(_pwm_pin, 0);
}
