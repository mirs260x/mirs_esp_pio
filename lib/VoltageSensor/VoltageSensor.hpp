#pragma once
#include <Arduino.h>

class VoltageSensor {
public:
    void begin(uint8_t pin, float divider_ratio);

    // 移動平均を取った電圧値[V]を返す
    float readVoltage();

private:
    uint8_t _pin = 0;
    float _divider_ratio = 1.0f;

    static constexpr int SAMPLE_NUM = 16;
    float _filtered = 0.0f;
    bool _initialized = false;
};
