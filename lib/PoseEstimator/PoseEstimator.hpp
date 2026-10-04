/**
 * @file PoseEstimator.hpp
 * @brief 状態推定ファサード（計算層）。エンコーダ差分＋IMU→pose/速度。
 * @details OdometryCalculator・VelocityCalculator・Mirs2605Ekf を束ね、
 *  control_task からは update()＋getter のみで使う。推定の判断はここに集約し、
 *  タスク側には配線（DiffDrive読取→投入→結果取出し）だけ残す。
 *  Arduino/FreeRTOS非依存のため host試験可（IMU入力は下記の値型で受ける）。
 */
#pragma once

#include <stdint.h>

#include "Mirs2605Ekf.hpp"
#include "OdometryCalculator.hpp"
#include "VelocityCalculator.hpp"

/** @brief 推定器へのIMU入力（値型・FW非依存）。
 *  @details control_task が SharedSensor から変換する（配線）。
 *  SystemContextへの依存を持ち込まないための境界型。 */
struct EstimatorImu {
    bool imu_ok = false;
    bool mag_ok = false;
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gz = 0.0f;
    float mx = 0.0f, my = 0.0f, mz = 0.0f;
};

class PoseEstimator {
public:
    // 単発スパイク棄却しきい値 [counts/周期]。1周期の真の移動は最大でも
    // 数百カウント（0.8m/s・15msで約73）のため、超過分は積算・速度更新しない。
    static constexpr int64_t kMaxDeltaPerCycle = 2048;

    PoseEstimator(double counts_per_rev, double wheel_radius, double wheel_base, bool use_ekf);

    /** @brief パラメータ適用（変更時のみ呼ぶこと）。 */
    void applyParams(double wheel_radius, double wheel_base);

    /** @brief pose・速度・累積をゼロ化する（`/reset_odometry` 用）。 */
    void reset();

    /**
     * @brief 状態推定1ステップ。
     * @param dl 前回からの左差分[カウント]（DiffDrive::sample()の戻り。そのまま渡すこと）
     * @param dr 同右
     * @param dist_l 左右移動距離[m]（DiffDrive::distLeft()。odom用）
     * @param dist_r 同右
     * @param imu IMU入力（不通時は imu_ok=false）
     * @param dt_sec 実測周期[s]
     */
    void update(int64_t dl, int64_t dr, double dist_l, double dist_r,
                const EstimatorImu &imu, double dt_sec);

    // 融合結果（EKF無効時はエンコーダオドメトリ）
    float x() const;
    float y() const;
    float theta() const;
    float v() const;  // 前進速度[m/s]（/odom twist用）
    float w() const;  // 角速度[rad/s]（同上）
    // PIDフィードバック用車輪速度[m/s]
    float velL() const;
    float velR() const;

private:
    void updateEkf(const EstimatorImu &imu, float v_enc, float w_enc, float dt_sec);

    OdometryCalculator odom_;
    Mirs2605Ekf ekf_;
    VelocityCalculator vel_calc_;
    bool use_ekf_;
    double wheel_base_;
    int64_t cum_l_ = 0;
    int64_t cum_r_ = 0;
    int64_t cum_prev_l_ = 0;
    int64_t cum_prev_r_ = 0;
    double vel_l_ = 0.0;
    double vel_r_ = 0.0;
    float w_ = 0.0f;
};
