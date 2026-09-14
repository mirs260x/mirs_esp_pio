/**
 * @file VelocityCalculator.cpp
 * @brief VelocityCalculatorクラスの実装
 */

#include "VelocityCalculator.hpp"

VelocityCalculator::VelocityCalculator(double counts_per_rev, double wheel_radius, double dt_sec)
    : counts_per_rev_(counts_per_rev)
    , wheel_radius_(wheel_radius)
    , dt_sec_(dt_sec)
{
}

void VelocityCalculator::setWheelRadius(double radius) {
    wheel_radius_ = radius;
}

void VelocityCalculator::setDeltaTime(double dt_sec) {
    dt_sec_ = dt_sec;
}

int64_t VelocityCalculator::wrapDelta(int32_t current_count, int32_t last_count) {
    // 符号付き減算のUBを避け、unsigned演算のラップで差分を求めて符号拡張する。
    // 1周期の真の移動量が2^31未満ならラップ時も正しい差分になる
    return static_cast<int64_t>(
        static_cast<int32_t>(static_cast<uint32_t>(current_count) - static_cast<uint32_t>(last_count)));
}

double VelocityCalculator::calculate(int64_t current_count, int64_t &prev_count) {
    const int64_t delta = current_count - prev_count;
    prev_count = current_count;

    // 速度 [m/s] = (カウント差 / カウント/回転) × 2π × 半径 / 時間
    return (static_cast<double>(delta) / counts_per_rev_) * 2.0 * PI * wheel_radius_ / dt_sec_;
}

double VelocityCalculator::calculate(int32_t current_count, int32_t &prev_count) {
    const int64_t delta = wrapDelta(current_count, prev_count);
    prev_count = current_count;

    return (static_cast<double>(delta) / counts_per_rev_) * 2.0 * PI * wheel_radius_ / dt_sec_;
}

void VelocityCalculator::calculateBothWheels(
    int64_t current_count_l, int64_t current_count_r,
    int64_t &prev_count_l, int64_t &prev_count_r,
    double &vel_l, double &vel_r
) {
    vel_l = calculate(current_count_l, prev_count_l);
    vel_r = calculate(current_count_r, prev_count_r);
}

void VelocityCalculator::calculateBothWheels(
    int32_t current_count_l, int32_t current_count_r,
    int32_t &prev_count_l, int32_t &prev_count_r,
    double &vel_l, double &vel_r
) {
    const int64_t dl = wrapDelta(current_count_l, prev_count_l);
    const int64_t dr = wrapDelta(current_count_r, prev_count_r);

    prev_count_l = current_count_l;
    prev_count_r = current_count_r;

    vel_l = (static_cast<double>(dl) / counts_per_rev_) * 2.0 * PI * wheel_radius_ / dt_sec_;
    vel_r = (static_cast<double>(dr) / counts_per_rev_) * 2.0 * PI * wheel_radius_ / dt_sec_;
}
