#pragma once
#include <stdint.h>
#include "Encoder.hpp"

/**
 * @brief 差動二輪オドメトリ計算器。
 * @details 左右エンコーダ（Encoder）の入力受付・反転補正・パルス差分からの移動距離変換、
 *          および車体位置 (x, y, theta) と最新速度 (v_linear, v_angular) の積算を一括で管理します。
 *          （旧DiffDriveとOdometryCalculatorを完全統合）
 */
class OdometryCalculator {
public:
    OdometryCalculator();
    OdometryCalculator(Encoder &left, Encoder &right,
                       bool left_reversed = false, bool right_reversed = false);

    void begin();
    void setWheelBase(double wheel_base);
    void setWheelParams(double wheel_radius, double wheel_base);
    void setReversed(bool left_reversed, bool right_reversed);
    void reset();

    /**
     * @brief 左右エンコーダからサンプリングし、移動距離・位置姿勢・速度を同時更新する。
     * @param dt_sec 制御周期 [s]
     */
    void update(double dt_sec = 0.015);

    /**
     * @brief 反転補正済み最新カウントスナップショットを取得する。
     */
    void snapshot(int32_t &count_l, int32_t &count_r) const {
        count_l = static_cast<int32_t>(left_ ? (left_reversed_ ? -left_->getCount() : left_->getCount()) : 0);
        count_r = static_cast<int32_t>(right_ ? (right_reversed_ ? -right_->getCount() : right_->getCount()) : 0);
    }

    /**
     * @brief 左右エンコーダからサンプリングし、差分・カウントを出力しつつオドメトリを更新する。
     * @param count_l 反転補正後左カウント (出力)
     * @param count_r 反転補正後右カウント (出力)
     * @param delta_l 前回からの差分カウント (出力)
     * @param delta_r 同右 (出力)
     * @param dt_sec  制御周期 [s]
     */
    void sampleAndUpdate(int32_t &count_l, int32_t &count_r,
                         int64_t &delta_l, int64_t &delta_r,
                         double dt_sec = 0.015);

    void sample(int32_t &count_l, int32_t &count_r,
                int64_t &delta_l, int64_t &delta_r) {
        sampleAndUpdate(count_l, count_r, delta_l, delta_r, 0.015);
    }

    /** @brief 外部から距離を与えてオドメトリを直接更新する場合（テスト・シミュレーション用） */
    void updateWithDistance(double dist_left, double dist_right, double dt_sec);
    void update(double dist_left, double dist_right, double dt_sec) {
        updateWithDistance(dist_left, dist_right, dt_sec);
    }

    // 積算位置・姿勢
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;

    // 最新速度
    float v_linear = 0.0f;
    float v_angular = 0.0f;

    // 直近ステップの左右移動距離 [m]
    double distLeft() const { return dist_l_; }
    double distRight() const { return dist_r_; }
    double wheelBase() const { return wheel_base_; }
    double wheelRadius() const { return wheel_radius_; }

private:
    Encoder *left_ = nullptr;
    Encoder *right_ = nullptr;
    bool left_reversed_ = false;
    bool right_reversed_ = false;

    int64_t last_l_ = 0;
    int64_t last_r_ = 0;

    double wheel_radius_ = 0.0391; // [m]
    double wheel_base_ = 0.39;     // [m]
    double dist_l_ = 0.0;
    double dist_r_ = 0.0;
};
