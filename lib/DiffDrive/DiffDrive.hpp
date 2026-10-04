#pragma once
#include <Arduino.h>
#include "Encoder.hpp"

/** @brief 差動二輪のインターフェース層。カウント差分→左右移動距離に変換する。
 *  @details 速度・姿勢の計算は持たない（計算層：VelocityCalculator・OdometryCalculatorの仕事）。 */
class DiffDrive {
public:
    DiffDrive(Encoder &left, Encoder &right,
              bool left_reversed = false, bool right_reversed = false);

    void begin();  // 保持するEncoder2つのbegin()を呼ぶ
    void setWheelParams(double wheel_radius, double wheel_base);
    void setReversed(bool left_reversed, bool right_reversed);
    // host試験専用。本番（control_task）はsample()を使うこと。
    void update();  // 差分を取り込み、移動距離を更新

    // host試験専用。本番はsample()のcount出力を使うこと。
    // VelocityCalculator等への受渡し用スナップショット（反転補正済み）
    void snapshot(int32_t &count_l, int32_t &count_r) const;

    /** @brief 単一読取でスナップショット・差分・移動距離を同時更新する。
     *  @details snapshot()+update()を分けると2回読みで値がずれるため、
     *  制御周期内の使用は本関数に一本化すること。
     *  差分は反転補正・ラップ吸収済みで、そのまま累積・速度計算に使える。
     *  二重積算防止のため、呼び出し側でwrapDeltaを掛け直してはならない。
     *  @param count_l 反転補正済み左カウント（出力。`/encoder` 表示用）
     *  @param count_r 反転補正済み右カウント（出力。同上）
     *  @param delta_l 前回sample()からの反転補正済み差分[カウント]（出力。累積用）
     *  @param delta_r 同右（出力）
     */
    void sample(int32_t &count_l, int32_t &count_r, int64_t &delta_l, int64_t &delta_r);

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
    double wheel_radius_ = 0.0391;  // 既定値はconfig.yamlと同一に保つこと
    double wheel_base_ = 0.39;      // （単一真実はyaml。初回/params受信で上書き）
    double dist_l_ = 0.0;
    double dist_r_ = 0.0;
};
