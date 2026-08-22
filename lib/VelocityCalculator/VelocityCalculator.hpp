/**
 * @file VelocityCalculator.hpp
 * @brief エンコーダーカウントから車輪速度を計算するクラス
 * 
 * エンコーダーの差分カウントから時間微分により速度[m/s]を計算します。
 */

#ifndef VELOCITY_CALCULATOR_HPP
#define VELOCITY_CALCULATOR_HPP

#include <Arduino.h>

class VelocityCalculator {
public:
    /**
     * @brief コンストラクタ
     * @param counts_per_rev エンコーダーの1回転あたりのカウント数
     * @param wheel_radius 車輪半径 [m]
     * @param dt_sec サンプリング周期 [s]
     */
    VelocityCalculator(double counts_per_rev, double wheel_radius, double dt_sec);

    /**
     * @brief 車輪半径を設定
     * @param radius 車輪半径 [m]
     */
    void setWheelRadius(double radius);

    /**
     * @brief サンプリング周期を設定
     * @param dt_sec サンプリング周期 [s]
     */
    void setDeltaTime(double dt_sec);

    /**
     * @brief 現在のエンコーダーカウントから速度を計算
     * @param current_count 現在のエンコーダーカウント
     * @param prev_count 前回のエンコーダーカウント（参照で更新される）
     * @return 計算された速度 [m/s]
     */
    double calculate(int32_t current_count, int32_t &prev_count);

    /**
     * @brief 左右両輪の速度を計算
     * @param current_count_l 現在の左輪エンコーダーカウント
     * @param current_count_r 現在の右輪エンコーダーカウント
     * @param prev_count_l 前回の左輪エンコーダーカウント（参照で更新される）
     * @param prev_count_r 前回の右輪エンコーダーカウント（参照で更新される）
     * @param vel_l 計算された左輪速度 [m/s]（出力）
     * @param vel_r 計算された右輪速度 [m/s]（出力）
     */
    void calculateBothWheels(
        int32_t current_count_l, int32_t current_count_r,
        int32_t &prev_count_l, int32_t &prev_count_r,
        double &vel_l, double &vel_r
    );

private:
    double counts_per_rev_;  ///< エンコーダー1回転あたりのカウント数
    double wheel_radius_;    ///< 車輪半径 [m]
    double dt_sec_;          ///< サンプリング周期 [s]
};

#endif // VELOCITY_CALCULATOR_HPP
