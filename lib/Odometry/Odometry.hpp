#pragma once

// 差動二輪オドメトリ。左右の移動距離から自己位置・姿勢を積算する。
// 入力は距離[m]のみでEncoder等に依存しないため、
// DifferentialDrive（実機）以外からの駆動・単体テストが容易。
class Odometry {
public:
    void setWheelBase(double wheel_base);
    void update(double dist_left, double dist_right, double dt_sec);
    void reset();

    // 積算位置・姿勢
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;

    // 最新速度（直近update()結果）
    float v_linear = 0.0f;
    float v_angular = 0.0f;

private:
    double wheel_base_ = 0.38;
};
