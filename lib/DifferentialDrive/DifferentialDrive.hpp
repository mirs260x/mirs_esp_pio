#pragma once
#include <Arduino.h>
#include "Encoder.hpp"

// 差動二輪の変換層。EncoderとOdometryの間に置くインターフェースで、
// エンコーダカウント差分を左右の移動距離・速度に変換する。
// 姿勢積算は持たない（Odometryの仕事）。
class DifferentialDrive {
public:
    DifferentialDrive(Encoder &left, Encoder &right);

    void begin();  // 保持するEncoder2つのbegin()を呼ぶ
    void setWheelParams(double wheel_radius, double wheel_base);
    void update(double dt_sec);  // 差分を取り込み、距離・速度を更新
    void reset();

    // VelocityCalculator等への受渡し用スナップショット
    void snapshot(int32_t &count_l, int32_t &count_r) const;

    // 最新ステップの移動距離 [m]・速度 [m/s]
    double distLeft() const { return dist_l_; }
    double distRight() const { return dist_r_; }
    double velLeft() const { return vel_l_; }
    double velRight() const { return vel_r_; }
    double wheelBase() const { return wheel_base_; }

private:
    Encoder &left_;
    Encoder &right_;
    int32_t last_l_ = 0;
    int32_t last_r_ = 0;
    double wheel_radius_ = 0.04;
    double wheel_base_ = 0.38;
    double dist_l_ = 0.0;
    double dist_r_ = 0.0;
    double vel_l_ = 0.0;
    double vel_r_ = 0.0;
};
