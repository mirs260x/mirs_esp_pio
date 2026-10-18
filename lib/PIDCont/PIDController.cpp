/**
 * @file PIDController.cpp
 * @brief PIDControllerクラスの実装
 */

#include "PIDController.hpp"

namespace {
// 出力制限。範囲外は端に丸める
double clampOutput(double value, double min_value, double max_value) {
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}
}  // namespace

PIDController::PIDController(double kp, double ki, double kd, double nominal_dt)
    : kp_(kp)
    , ki_(ki)
    , kd_(kd)
    , nominal_dt_(nominal_dt)
    , err_sum_(0.0)
    , prev_err_(0.0)
    , output_min_(-255.0)
    , output_max_(255.0)
{
}

void PIDController::setGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setNominalDt(double nominal_dt) {
    nominal_dt_ = nominal_dt;
}

void PIDController::setOutputLimits(double min, double max) {
    output_min_ = min;
    output_max_ = max;
}

double PIDController::compute(double setpoint, double measured, double dt_sec) {
    // 誤差計算
    const double error = setpoint - measured;

    // dt異常時 (<=0) はP+保持Iのみ出力し、積分・微分状態を凍結する
    if (dt_sec <= 0.0) {
        return clampOutput(kp_ * error + ki_ * err_sum_, output_min_, output_max_);
    }

    // スコープ内ローカルでdtを公称周期の1/3〜3倍にクランプする。
    // これによりゲインの効きが更新周期に依存しなくなる
    double dt = dt_sec;
    if (nominal_dt_ > 0.0) {
        const double dt_min = nominal_dt_ / 3.0;
        const double dt_max = nominal_dt_ * 3.0;
        if (dt < dt_min) {
            dt = dt_min;
        } else if (dt > dt_max) {
            dt = dt_max;
        }
    }
    const double p = kp_ * error;
    // I項の候補 (条件付き積分でアンチワインドアップ。dt正規化済み)
    const double trial_sum = err_sum_ + error * dt;
    const double i = ki_ * trial_sum;
    const double d = kd_ * (error - prev_err_) / dt;
    const double trial_out = p + i + d;

    // 出力制限
    const double output = clampOutput(trial_out, output_min_, output_max_);

    // 飽和していない、または飽和を緩和する方向なら積分を採用。
    // 飽和を悪化させる方向（errorと出力が同符号）のみ積分を凍結する
    if (output == trial_out || (error * trial_out) < 0.0) {
        err_sum_ = trial_sum;
    }

    // 前回誤差を保存
    prev_err_ = error;

    return output;
}

void PIDController::reset() {
    err_sum_ = 0.0;
    prev_err_ = 0.0;
}
