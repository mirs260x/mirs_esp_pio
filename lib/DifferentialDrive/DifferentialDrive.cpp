#include "DifferentialDrive.hpp"
#include <math.h>

DifferentialDrive::DifferentialDrive(Encoder &left, Encoder &right)
    : left_(left)
    , right_(right)
{
}

void DifferentialDrive::begin() {
    left_.begin();
    right_.begin();
}

void DifferentialDrive::setWheelParams(double wheel_radius, double wheel_base) {
    wheel_radius_ = wheel_radius;
    wheel_base_ = wheel_base;
}

void DifferentialDrive::update(double dt_sec) {
    const int32_t cur_l = left_.getCount();
    const int32_t cur_r = right_.getCount();
    const int32_t dl = cur_l - last_l_;
    const int32_t dr = cur_r - last_r_;
    last_l_ = cur_l;
    last_r_ = cur_r;

    const double cpr = static_cast<double>(left_.countsPerRev());
    const double dist_l = (dl / cpr) * 2.0 * M_PI * wheel_radius_;
    const double dist_r = (dr / cpr) * 2.0 * M_PI * wheel_radius_;

    const double d = (dist_l + dist_r) / 2.0;
    const double dtheta = (dist_r - dist_l) / wheel_base_;

    theta += static_cast<float>(dtheta);
    x += static_cast<float>(d * cos(theta));
    y += static_cast<float>(d * sin(theta));

    if (dt_sec > 0.0) {
        v_linear = static_cast<float>(d / dt_sec);
        v_angular = static_cast<float>(dtheta / dt_sec);
    }
}

void DifferentialDrive::reset() {
    last_l_ = left_.getCount();
    last_r_ = right_.getCount();
    x = y = theta = 0.0f;
    v_linear = v_angular = 0.0f;
}

void DifferentialDrive::snapshot(int32_t &count_l, int32_t &count_r) const {
    count_l = left_.getCount();
    count_r = right_.getCount();
}
