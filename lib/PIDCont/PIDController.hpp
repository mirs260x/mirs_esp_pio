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

/** @brief 汎用PID制御 (正規形。ゲインは周期不変)。 */
class PIDController {
public:
    /**
     * @brief コンストラクタ
     * @param kp 比例ゲイン
     * @param ki 積分ゲイン
     * @param kd 微分ゲイン
     * @param nominal_dt 公称制御周期 [s]。dtクランプ範囲 (1/3〜3倍) の基準
     */
    PIDController(double kp, double ki, double kd, double nominal_dt = 0.015);

    /**
     * @brief PIDゲインを設定
     * @param kp 比例ゲイン
     * @param ki 積分ゲイン
     * @param kd 微分ゲイン
     */
    void setGains(double kp, double ki, double kd);

    /**
     * @brief 公称制御周期を設定
     * @param nominal_dt 公称制御周期 [s] (>0)。dtクランプ範囲の基準
     */
    void setNominalDt(double nominal_dt);

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
     * @param dt_sec 制御周期 [s]（<=0ではP+保持Iのみ出力し積分・微分を凍結。
     *               公称周期の1/3〜3倍にクランプ）
     * @return 制御出力（setOutputLimits範囲に制限済み）
     *
     * 正規形 (P + I*dt累積 + D/dt)。ゲインは周期不変:
     *   P = Kp*err, I = Ki*Σ(err*dt), D = Kd*Δerr/dt。
     * 他式で調整した値を流用する場合は換算すること:
     *   Ti=Kp*dt/Ki_raw, Td=Kd_raw*dt/Kp。
     * アンチワインドアップ：出力飽和を悪化させる方向の積分だけ凍結する
     * （条件付き積分）。
     */
    double compute(double setpoint, double measured, double dt_sec);

    /**
     * @brief PID制御状態をリセット
     * 
     * 積分項と前回誤差をゼロクリアします。
     * モーター停止時などに呼び出してください。
     */
    void reset();

    /**
     * @brief 現在の積分項を取得
     * @return 積分項の値（∫err*dt の累積。単位は [err*s]）
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
    double nominal_dt_;  ///< 公称制御周期 [s]（dtクランプ範囲の基準）
    double err_sum_;     ///< 積分項（誤差の累積）
    double prev_err_;    ///< 前回の誤差（微分計算用）
    double output_min_;  ///< 出力最小値
    double output_max_;  ///< 出力最大値
};

#endif // PID_CONTROLLER_HPP
