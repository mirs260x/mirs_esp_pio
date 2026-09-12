#pragma once
#include <Arduino.h>
#include "Encoder.hpp"

namespace mirs2605 {

// 差動二輪のインターフェース層。EncoderとOdometryの間に置き、
// エンコーダカウント差分を左右の移動距離に変換する。
// 速度・姿勢の計算は持たない（計算層：VelocityCalculator・Odometryの仕事）。
class DiffDrive {
public:
    DiffDrive(Encoder &left, Encoder &right,
              bool left_reversed = false, bool right_reversed = false);

    void begin();  // 保持するEncoder2つのbegin()を呼ぶ
    void setWheelParams(double wheel_radius, double wheel_base);
    void setReversed(bool left_reversed, bool right_reversed);
    void update();  // 差分を取り込み、移動距離を更新
    void reset();

    // VelocityCalculator等への受渡し用スナップショット（反転補正済み）
    void snapshot(int32_t &count_l, int32_t &count_r) const;

    // 最新ステップの移動距離 [m]
    double distLeft() const { return dist_l_; }
    double distRight() const { return dist_r_; }
    double wheelBase() const { return wheel_base_; }

private:
    Encoder &left_;
    Encoder &right_;
    bool left_reversed_;
    bool right_reversed_;
    int32_t last_l_ = 0;
    int32_t last_r_ = 0;
    double wheel_radius_ = 0.04;
    double wheel_base_ = 0.38;
    double dist_l_ = 0.0;
    double dist_r_ = 0.0;
};


}  // namespace mirs2605