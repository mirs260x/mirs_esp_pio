#include "Odometry.hpp"
#include <math.h>

namespace mirs2605 {

namespace {

// [-PI, PI] に正規化する。長時間運転でのfloat精度劣化を防ぐ。
float normalizeAngle(float angle) {
    return atan2f(sinf(angle), cosf(angle));
}

}  // namespace

void Odometry::setWheelBase(double wheel_base) {
    wheel_base_ = wheel_base;
}

void Odometry::update(double dist_left, double dist_right, double dt_sec) {
    const double d = (dist_left + dist_right) / 2.0;
    const double dtheta = (dist_right - dist_left) / wheel_base_;

    // 中点法：ステップ中間の姿勢で並進を積分し、円弧誤差を低減する。
    // （更新後thetaで積分すると曲線走行で内回りバイアスが溜まる）
    const double mid = static_cast<double>(theta) + dtheta / 2.0;
    theta = normalizeAngle(static_cast<float>(static_cast<double>(theta) + dtheta));
    x += static_cast<float>(d * cos(mid));
    y += static_cast<float>(d * sin(mid));

    if (dt_sec > 0.0) {
        v_linear = static_cast<float>(d / dt_sec);
        v_angular = static_cast<float>(dtheta / dt_sec);
    }
}

void Odometry::reset() {
    x = y = theta = 0.0f;
    v_linear = v_angular = 0.0f;
}

}  // namespace mirs2605