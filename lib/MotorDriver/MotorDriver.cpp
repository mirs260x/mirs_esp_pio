#include "MotorDriver.hpp"
#include <stdlib.h>

namespace mirs2605 {

MotorDriver::MotorDriver(uint8_t pin_pwm, uint8_t pin_dir)
    : pin_pwm_(pin_pwm)
    , pin_dir_(pin_dir)
{
}

void MotorDriver::begin(uint32_t pwm_freq, uint8_t pwm_resolution) {
    pinMode(pin_dir_, OUTPUT);
    ledcAttach(pin_pwm_, pwm_freq, pwm_resolution);
    stop();
}

void MotorDriver::setDuty(int duty) {
    if (duty > DUTY_MAX) {
        duty = DUTY_MAX;
    } else if (duty < -DUTY_MAX) {
        duty = -DUTY_MAX;
    }
    digitalWrite(pin_dir_, duty >= 0 ? HIGH : LOW);
    ledcWrite(pin_pwm_, static_cast<uint8_t>(abs(duty)));
}

void MotorDriver::stop() {
    ledcWrite(pin_pwm_, 0);
}

}  // namespace mirs2605