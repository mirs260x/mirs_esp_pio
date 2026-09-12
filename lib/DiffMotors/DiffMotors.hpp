#pragma once
#include "MotorDriver.hpp"

namespace mirs2605 {

// 差動用モータペアのインターフェース層。正逆極性の吸収が仕事。
// 速度→dutyの計算は持たない（PIDControllerの仕事）。
//
// ハード変更（モータ極性反転等）があればreverse指定だけ変える。
// 既定値（右反転）は現行MotorControllerのDIR論理と一致。
class DiffMotors {
public:
    DiffMotors(MotorDriver &left, MotorDriver &right,
                       bool left_reversed = false, bool right_reversed = true);

    void begin(uint32_t pwm_freq = 20000, uint8_t pwm_resolution = 8);
    void setReversed(bool left_reversed, bool right_reversed);
    void setLeft(double duty);
    void setRight(double duty);
    void setBoth(double duty_left, double duty_right);
    void stop();

private:
    MotorDriver &left_;
    MotorDriver &right_;
    bool left_reversed_;
    bool right_reversed_;
};

}  // namespace mirs2605