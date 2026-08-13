#pragma once
#include <ESP32Encoder.h>

class Odometry {
public:
    void begin(uint8_t enc_l_a, uint8_t enc_l_b, uint8_t enc_r_a, uint8_t enc_r_b);

    // 一定周期(dt秒)で呼び出す
    void update(float dt);

    void reset();

    // 現在の位置・姿勢
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;

    // 現在の速度 (最新のupdate()結果)
    float v_linear = 0.0f;
    float v_angular = 0.0f;

private:
    ESP32Encoder _enc_l;
    ESP32Encoder _enc_r;
    int64_t _last_count_l = 0;
    int64_t _last_count_r = 0;
};
