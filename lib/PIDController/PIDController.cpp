/**
 * @file PIDController.cpp
 * @brief PIDControllerクラスの実装
 */

#include "PIDController.hpp"

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp)
    , ki_(ki)
    , kd_(kd)
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

void PIDController::setOutputLimits(double min, double max) {
    output_min_ = min;
    output_max_ = max;
}

double PIDController::compute(double setpoint, double measured) {
    // 誤差計算
    double error = setpoint - measured;
    
    // 積分項更新
    err_sum_ += error;
    
    // PID計算
    double output = kp_ * error + ki_ * err_sum_ + kd_ * (error - prev_err_);
    
    // 前回誤差を保存
    prev_err_ = error;
    
    // 出力制限
    if (output > output_max_) {
        output = output_max_;
    } else if (output < output_min_) {
        output = output_min_;
    }
    
    return output;
}

void PIDController::reset() {
    err_sum_ = 0.0;
    prev_err_ = 0.0;
}
