/**
 * @file PIDController.hpp
 * @brief 汎用PID制御クラス
 * 
 * 比例(P)・積分(I)・微分(D)制御を行うクラス。
 * 目標値と現在値の差分から制御出力を計算します。
 */

#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <Arduino.h>

class PIDController {
public:
    /**
     * @brief コンストラクタ
     * @param kp 比例ゲイン
     * @param ki 積分ゲイン
     * @param kd 微分ゲイン
     */
    PIDController(double kp, double ki, double kd);

    /**
     * @brief PIDゲインを設定
     * @param kp 比例ゲイン
     * @param ki 積分ゲイン
     * @param kd 微分ゲイン
     */
    void setGains(double kp, double ki, double kd);

    /**
     * @brief 出力リミットを設定
     * @param min 最小出力値
     * @param max 最大出力値
     */
    void setOutputLimits(double min, double max);

    /**
     * @brief PID制御計算を実行
     * @param setpoint 目標値
     * @param measured 現在値（測定値）
     * @return 制御出力
     */
    double compute(double setpoint, double measured);

    /**
     * @brief PID制御状態をリセット
     * 
     * 積分項と前回誤差をゼロクリアします。
     * モーター停止時などに呼び出してください。
     */
    void reset();

    /**
     * @brief 現在の積分項を取得
     * @return 積分項の値
     */
    double getIntegralTerm() const { return err_sum_; }

    /**
     * @brief 現在の前回誤差を取得
     * @return 前回誤差の値
     */
    double getPrevError() const { return prev_err_; }

private:
    double kp_;          ///< 比例ゲイン
    double ki_;          ///< 積分ゲイン
    double kd_;          ///< 微分ゲイン
    double err_sum_;     ///< 積分項（誤差の累積）
    double prev_err_;    ///< 前回の誤差（微分計算用）
    double output_min_;  ///< 出力最小値
    double output_max_;  ///< 出力最大値
};

#endif // PID_CONTROLLER_HPP
