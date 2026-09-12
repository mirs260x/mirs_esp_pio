#include "DifferentialMotors.hpp"

DifferentialMotors::DifferentialMotors(MotorDriver &left, MotorDriver &right,
                                       bool left_reversed, bool right_reversed)
    : left_(left)
    , right_(right)
    , left_reversed_(left_reversed)
    , right_reversed_(right_reversed)
{
}

void DifferentialMotors::begin(uint32_t pwm_freq, uint8_t pwm_resolution) {
    left_.begin(pwm_freq, pwm_resolution);
    right_.begin(pwm_freq, pwm_resolution);
}

void DifferentialMotors::setReversed(bool left_reversed, bool right_reversed) {
    left_reversed_ = left_reversed;
    right_reversed_ = right_reversed;
}

void DifferentialMotors::setLeft(double duty) {
    left_.setDuty(static_cast<int>(left_reversed_ ? -duty : duty));
}

void DifferentialMotors::setRight(double duty) {
    right_.setDuty(static_cast<int>(right_reversed_ ? -duty : duty));
}

void DifferentialMotors::setBoth(double duty_left, double duty_right) {
    setLeft(duty_left);
    setRight(duty_right);
}

void DifferentialMotors::stop() {
    left_.stop();
    right_.stop();
}
