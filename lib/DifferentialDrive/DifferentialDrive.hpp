#pragma once
#include <Arduino.h>
#include "Encoder.hpp"

// 差動二輪を表すクラス。Encoderオブジェクト2つを保持し（所有は呼び出し側）、
// 左右カウント差分から自己位置・姿勢を積算する。
// 運動学はROS側 odometry_publisher と同一式。
class DifferentialDrive {
public:
    DifferentialDrive(Encoder &left, Encoder &right);

    void begin();  // 保持するEncoder2つのbegin()を呼ぶ
    void setWheelParams(double wheel_radius, double wheel_base);
    void update(double dt_sec);  // カウント差分を取り込み、x/y/theta/vを更新
    void reset();

    // VelocityCalculator等への受渡し用スナップショット
    void snapshot(int32_t &count_l, int32_t &count_r) const;

    // 積算位置・姿勢
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;

    // 最新速度（直近update()結果）
    float v_linear = 0.0f;
    float v_angular = 0.0f;

private:
    Encoder &left_;
    Encoder &right_;
    int32_t last_l_ = 0;
    int32_t last_r_ = 0;
    double wheel_radius_ = 0.04;
    double wheel_base_ = 0.38;
};
