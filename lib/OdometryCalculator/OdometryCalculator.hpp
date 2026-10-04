#pragma once

/** @brief 差動二輪オドメトリ。左右移動距離[m]から自己位置・姿勢を積算する。
 *  @details 入力は距離のみでEncoder等に依存しない。 */
class OdometryCalculator {
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
    // 既定値はconfig.yamlと同一に保つこと（単一真実はyaml）
    double wheel_base_ = 0.39;
};
