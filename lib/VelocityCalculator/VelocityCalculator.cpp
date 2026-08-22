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

double VelocityCalculator::calculate(int32_t current_count, int32_t &prev_count) {
    const int32_t delta = current_count - prev_count;
    prev_count = current_count;
    
    // 速度 [m/s] = (カウント差 / カウント/回転) × 2π × 半径 / 時間
    return (delta / counts_per_rev_) * 2.0 * PI * wheel_radius_ / dt_sec_;
}

void VelocityCalculator::calculateBothWheels(
    int32_t current_count_l, int32_t current_count_r,
    int32_t &prev_count_l, int32_t &prev_count_r,
    double &vel_l, double &vel_r
) {
    const int32_t dl = current_count_l - prev_count_l;
    const int32_t dr = current_count_r - prev_count_r;
    
    prev_count_l = current_count_l;
    prev_count_r = current_count_r;
    
    vel_l = (dl / counts_per_rev_) * 2.0 * PI * wheel_radius_ / dt_sec_;
    vel_r = (dr / counts_per_rev_) * 2.0 * PI * wheel_radius_ / dt_sec_;
}
