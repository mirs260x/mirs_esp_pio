#include "DiffMotors.hpp"

DiffMotors::DiffMotors(MotorDriver &left, MotorDriver &right,
                                       bool left_reversed, bool right_reversed)
    : left_(left)
    , right_(right)
    , left_reversed_(left_reversed)
    , right_reversed_(right_reversed)
{
}

void DiffMotors::begin(uint32_t pwm_freq, uint8_t pwm_resolution) {
    left_.begin(pwm_freq, pwm_resolution);
    right_.begin(pwm_freq, pwm_resolution);
}

void DiffMotors::setReversed(bool left_reversed, bool right_reversed) {
    left_reversed_ = left_reversed;
    right_reversed_ = right_reversed;
}

namespace {
// 四捨五入（切捨てでは微小dutyが消失する。MotorDriver側で±255にclampされる）
int roundDuty(double duty) {
    const int truncated = static_cast<int>(duty);
    const double frac = duty - truncated;
    if (frac >= 0.5) {
        return truncated + 1;
    }
    if (frac <= -0.5) {
        return truncated - 1;
    }
    return truncated;
}
}  // namespace

void DiffMotors::setLeft(double duty) {
    left_.setDuty(roundDuty(left_reversed_ ? -duty : duty));
}

void DiffMotors::setRight(double duty) {
    right_.setDuty(roundDuty(right_reversed_ ? -duty : duty));
}

void DiffMotors::setBoth(double duty_left, double duty_right) {
    setLeft(duty_left);
    setRight(duty_right);
}

void DiffMotors::stop() {
    left_.stop();
    right_.stop();
}
