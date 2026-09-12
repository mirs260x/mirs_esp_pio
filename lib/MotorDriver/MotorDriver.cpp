#include "MotorDriver.hpp"

// 外部(main.cpp)で定義されるE-Stopフラグ
extern volatile bool g_estop_active;

MotorDriver::MotorDriver(uint8_t pwm_pin, uint8_t dir_pin)
	:_pwm_pin(pwm_pin), _dir_pin(dir_pin) {}

void MotorDriver::begin() {
    pinMode(_dir_pin, OUTPUT);
    ledcAttach(_pwm_pin, PWM_FREQENCY, PWM_RESOLUTION);
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
