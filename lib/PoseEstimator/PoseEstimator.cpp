#include "PoseEstimator.hpp"

#include <math.h>

namespace {

// 地磁気外れ値棄却しきい値[rad]。屋内磁気外乱に備え残差大は棄却する。
constexpr float kMagGateRad = 1.0f;
// バイアス更新の低速ゲート[m/s]。走行中のエンコーダノイズ混入を防ぐ。
constexpr float kBiasGateVel = 0.05f;

}  // namespace

PoseEstimator::PoseEstimator(double counts_per_rev, double wheel_radius, double wheel_base, bool use_ekf)
    : vel_calc_(counts_per_rev, wheel_radius, 0.015)
    , use_ekf_(use_ekf)
    , wheel_base_(wheel_base)
{
    odom_.setWheelBase(wheel_base);
}

void PoseEstimator::applyParams(double wheel_radius, double wheel_base) {
    vel_calc_.setWheelRadius(wheel_radius);
    odom_.setWheelBase(wheel_base);
    wheel_base_ = wheel_base;
}

void PoseEstimator::reset() {
    odom_.reset();
    ekf_.reset();
    cum_l_ = cum_r_ = 0;
    cum_prev_l_ = cum_prev_r_ = 0;
    vel_l_ = vel_r_ = 0.0;
    w_ = 0.0f;
}

void PoseEstimator::update(int64_t dl, int64_t dr, double dist_l, double dist_r,
                           const EstimatorImu &imu, double dt_sec) {
    cum_l_ += dl;
    cum_r_ += dr;

    // 単発スパイクは参照だけ進めて積算・速度を凍結する。
    // calculateBothWheels は前回値を常に進めるため、次周期に跳躍を持ち越さない。
    const bool glitch = (dl > kMaxDeltaPerCycle || dl < -kMaxDeltaPerCycle ||
                         dr > kMaxDeltaPerCycle || dr < -kMaxDeltaPerCycle);

    vel_calc_.setDeltaTime(dt_sec);
    double tmp_l = vel_l_;
    double tmp_r = vel_r_;
    vel_calc_.calculateBothWheels(cum_l_, cum_r_, cum_prev_l_, cum_prev_r_, tmp_l, tmp_r);
    if (glitch) {
        return;
    }
    vel_l_ = tmp_l;
    vel_r_ = tmp_r;

    const float v_enc = static_cast<float>((vel_l_ + vel_r_) * 0.5);
    float w_enc = 0.0f;
    if (wheel_base_ > 1e-9) {
        w_enc = static_cast<float>((vel_r_ - vel_l_) / wheel_base_);
    }

    if (use_ekf_) {
        // EKF有効時はpose/velocityをEKFが出すためOdometryCalculatorは温存する
        updateEkf(imu, v_enc, w_enc, static_cast<float>(dt_sec));
    } else {
        odom_.update(dist_l, dist_r, dt_sec);
        w_ = odom_.v_angular;
    }
}

void PoseEstimator::updateEkf(const EstimatorImu &imu, float v_enc, float w_enc, float dt_sec) {
    // ジャイロ不通時はエンコーダ角速度で代用する（バイアス更新は行わない）
    const float gz = imu.imu_ok ? imu.gz : w_enc;
    ekf_.predict(gz, dt_sec);
    ekf_.updateVel(v_enc);
    if (imu.imu_ok) {
        ekf_.updateYawRateGated(w_enc, imu.gz, v_enc, kBiasGateVel);
        // 停止検出時は v=0 拘束で加速度計を活用する（ZUPT）
        if (Mirs2605Ekf::isStationary(imu.ax, imu.ay, imu.az, imu.gz, v_enc)) {
            ekf_.updateZeroVel();
        }
    }
    if (imu.mag_ok) {
        const float yaw_mag = Mirs2605Ekf::magYawTiltCompensated(
            imu.mx, imu.my, imu.mz, imu.ax, imu.ay, imu.az);
        if (fabsf(Mirs2605Ekf::angleResidual(yaw_mag, ekf_.theta())) < kMagGateRad) {
            ekf_.updateMagTheta(yaw_mag);
        }
    }
    w_ = imu.imu_ok ? (imu.gz - ekf_.gyroBias()) : w_enc;
}

float PoseEstimator::x() const {
    return use_ekf_ ? ekf_.x() : odom_.x;
}

float PoseEstimator::y() const {
    return use_ekf_ ? ekf_.y() : odom_.y;
}

float PoseEstimator::theta() const {
    return use_ekf_ ? ekf_.theta() : odom_.theta;
}

float PoseEstimator::v() const {
    return use_ekf_ ? ekf_.v() : odom_.v_linear;
}

float PoseEstimator::w() const {
    return w_;
}

float PoseEstimator::velL() const {
    return static_cast<float>(vel_l_);
}

float PoseEstimator::velR() const {
    return static_cast<float>(vel_r_);
}
