#pragma once
#include "MotorDriver.hpp"

/** @brief 差動用モータペアのインターフェース層。正逆極性の吸収が仕事。
 *  @details 速度→dutyの計算は持たない（PIDControllerの仕事）。
 *  ハード変更時はreverse指定だけ変える。既定値（右反転）はDIR論理と一致。 */
class DiffMotors {
public:
    DiffMotors(MotorDriver &left, MotorDriver &right,
                       bool left_reversed = false, bool right_reversed = true);

    void begin(uint32_t pwm_freq = 20000, uint8_t pwm_resolution = 8);
    void setReversed(bool left_reversed, bool right_reversed);
    void setLeft(double duty);
    void setRight(double duty);
    void setBoth(double duty_left, double duty_right);
    void stop();  // 本番未使用（非常停止なし方針。試験・将来用に維持）

private:
    MotorDriver &left_;
    MotorDriver &right_;
    bool left_reversed_;
    bool right_reversed_;
};
