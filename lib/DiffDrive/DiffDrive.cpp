#include "DiffDrive.hpp"
#include <math.h>

DiffDrive::DiffDrive(Encoder &left, Encoder &right,
                                       bool left_reversed, bool right_reversed)
    : left_(left)
    , right_(right)
    , left_reversed_(left_reversed)
    , right_reversed_(right_reversed)
{
}

void DiffDrive::begin() {
    left_.begin();
    right_.begin();
}

void DiffDrive::setWheelParams(double wheel_radius, double wheel_base) {
    wheel_radius_ = wheel_radius;
    wheel_base_ = wheel_base;
}

void DiffDrive::setReversed(bool left_reversed, bool right_reversed) {
    left_reversed_ = left_reversed;
    right_reversed_ = right_reversed;
}

void DiffDrive::update() {
    const int32_t cur_l = left_.getCount();
    const int32_t cur_r = right_.getCount();
    int32_t dl = cur_l - last_l_;
    int32_t dr = cur_r - last_r_;
    last_l_ = cur_l;
    last_r_ = cur_r;

    // ハード変更のソフト吸収：符号反転はこの層で完結させる
    if (left_reversed_) {
        dl = -dl;
    }
    if (right_reversed_) {
        dr = -dr;
    }

    const double cpr = static_cast<double>(left_.countsPerRev());
    dist_l_ = (dl / cpr) * 2.0 * M_PI * wheel_radius_;
    dist_r_ = (dr / cpr) * 2.0 * M_PI * wheel_radius_;
}

void DiffDrive::reset() {
    last_l_ = left_.getCount();
    last_r_ = right_.getCount();
    dist_l_ = dist_r_ = 0.0;
}

void DiffDrive::snapshot(int32_t &count_l, int32_t &count_r) const {
    // update()と同一の反転補正を適用する
    count_l = left_reversed_ ? -left_.getCount() : left_.getCount();
    count_r = right_reversed_ ? -right_.getCount() : right_.getCount();
}
