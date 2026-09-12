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
    dist_l_ = (dl / cpr) * 2.0 * M_PI * wheel_radius_;
    dist_r_ = (dr / cpr) * 2.0 * M_PI * wheel_radius_;

    if (dt_sec > 0.0) {
        vel_l_ = dist_l_ / dt_sec;
        vel_r_ = dist_r_ / dt_sec;
    }
}

void DifferentialDrive::reset() {
    last_l_ = left_.getCount();
    last_r_ = right_.getCount();
    dist_l_ = dist_r_ = 0.0;
    vel_l_ = vel_r_ = 0.0;
}

void DifferentialDrive::snapshot(int32_t &count_l, int32_t &count_r) const {
    count_l = left_.getCount();
    count_r = right_.getCount();
}
