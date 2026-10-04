/**
 * @file VelocityCalculator.hpp
 * @brief エンコーダーカウントから車輪速度を計算するクラス
 * 
 * エンコーダーの差分カウントから時間微分により速度[m/s]を計算します。
 */

#ifndef VELOCITY_CALCULATOR_HPP
#define VELOCITY_CALCULATOR_HPP

#include <Arduino.h>

/** @brief エンコーダカウント→車輪速度[m/s]の時間微分計算。 */
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
     * @brief int32境界のラップを吸収したカウント差分を返す
     * @param current_count 現在のカウント
     * @param last_count 前回のカウント
     * @return current - last を [-2^31, 2^31) で解釈した差分（int64）。
     * さらに16bit幅への畳み込みでHWカウンタ直読時の折返しにも備える。
     * 1制御周期の真の移動は数百カウント以下であることが前提。
     */
    static int64_t wrapDelta(int32_t current_count, int32_t last_count);

    /**
     * @brief ラップ吸収済み累積カウントから速度を計算（int64版）
     * @param current_count 現在の累積カウント
     * @param prev_count 前回の累積カウント（参照で更新される）
     * @return 計算された速度 [m/s]
     */
    double calculate(int64_t current_count, int64_t &prev_count);

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

    /**
     * @brief 左右両輪の速度を計算（int64累積値版）
     * @param current_count_l 現在の左輪累積カウント
     * @param current_count_r 現在の右輪累積カウント
     * @param prev_count_l 前回の左輪累積カウント（参照で更新される）
     * @param prev_count_r 前回の右輪累積カウント（参照で更新される）
     * @param vel_l 計算された左輪速度 [m/s]（出力）
     * @param vel_r 計算された右輪速度 [m/s]（出力）
     */
    void calculateBothWheels(
        int64_t current_count_l, int64_t current_count_r,
        int64_t &prev_count_l, int64_t &prev_count_r,
        double &vel_l, double &vel_r
    );

private:
    double counts_per_rev_;  ///< エンコーダー1回転あたりのカウント数
    double wheel_radius_;    ///< 車輪半径 [m]
    double dt_sec_;          ///< サンプリング周期 [s]
};

#endif // VELOCITY_CALCULATOR_HPP
