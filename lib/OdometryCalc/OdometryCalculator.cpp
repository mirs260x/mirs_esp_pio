#include "OdometryCalculator.hpp"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {
// [-PI, PI] に正規化する
float normalizeAngle(float angle) {
    return atan2f(sinf(angle), cosf(angle));
}
} // namespace

OdometryCalculator::OdometryCalculator()
    : left_(nullptr), right_(nullptr)
    , left_reversed_(false), right_reversed_(false)
{
}

OdometryCalculator::OdometryCalculator(Encoder &left, Encoder &right,
                                       bool left_reversed, bool right_reversed)
    : left_(&left), right_(&right)
    , left_reversed_(left_reversed), right_reversed_(right_reversed)
{
}

void OdometryCalculator::begin() {
    if (left_) left_->begin();
    if (right_) right_->begin();
    if (left_) last_l_ = left_->getCount();
    if (right_) last_r_ = right_->getCount();
}

void OdometryCalculator::setWheelBase(double wheel_base) {
    wheel_base_ = wheel_base;
}

void OdometryCalculator::setWheelParams(double wheel_radius, double wheel_base) {
    wheel_radius_ = wheel_radius;
    wheel_base_ = wheel_base;
}

void OdometryCalculator::setReversed(bool left_reversed, bool right_reversed) {
    left_reversed_ = left_reversed;
    right_reversed_ = right_reversed;
}

void OdometryCalculator::reset() {
    if (left_) left_->reset();
    if (right_) right_->reset();
    last_l_ = 0;
    last_r_ = 0;
    x = 0.0f;
    y = 0.0f;
    theta = 0.0f;
    v_linear = 0.0f;
    v_angular = 0.0f;
    dist_l_ = 0.0;
    dist_r_ = 0.0;
}

void OdometryCalculator::updateWithDistance(double dist_left, double dist_right, double dt_sec) {
    dist_l_ = dist_left;
    dist_r_ = dist_right;

    const double d = (dist_left + dist_right) / 2.0;
    const double dtheta = (dist_right - dist_left) / wheel_base_;

    // 中点法（Runge-Kutta 2次相当）: ステップ中間の姿勢で並進を積分
    const double mid = static_cast<double>(theta) + dtheta / 2.0;
    theta = normalizeAngle(static_cast<float>(static_cast<double>(theta) + dtheta));
    x += static_cast<float>(d * cos(mid));
    y += static_cast<float>(d * sin(mid));

    if (dt_sec > 0.0) {
        v_linear = static_cast<float>(d / dt_sec);
        v_angular = static_cast<float>(dtheta / dt_sec);
    }
}

void OdometryCalculator::update(double dt_sec) {
    int32_t cl = 0, cr = 0;
    int64_t dl = 0, dr = 0;
    sampleAndUpdate(cl, cr, dl, dr, dt_sec);
}

void OdometryCalculator::sampleAndUpdate(int32_t &count_l, int32_t &count_r,
                                         int64_t &delta_l, int64_t &delta_r,
                                         double dt_sec) {
    if (!left_ || !right_) {
        return;
    }
    const int64_t raw_l = left_->getCount();
    const int64_t raw_r = right_->getCount();

    int64_t dl = raw_l - last_l_;
    int64_t dr = raw_r - last_r_;
    last_l_ = raw_l;
    last_r_ = raw_r;

    // ハードウェア取付方向の反転補正
    if (left_reversed_) {
        dl = -dl;
    }
    if (right_reversed_) {
        dr = -dr;
    }

    delta_l = dl;
    delta_r = dr;
    count_l = static_cast<int32_t>(left_reversed_ ? -raw_l : raw_l);
    count_r = static_cast<int32_t>(right_reversed_ ? -raw_r : raw_r);

    const double cpr_l = static_cast<double>(left_->countsPerRev());
    const double cpr_r = static_cast<double>(right_->countsPerRev());

    const double dist_left = (dl / cpr_l) * 2.0 * M_PI * wheel_radius_;
    const double dist_right = (dr / cpr_r) * 2.0 * M_PI * wheel_radius_;

    updateWithDistance(dist_left, dist_right, dt_sec);
}
