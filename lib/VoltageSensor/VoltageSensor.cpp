#include "VoltageSensor.hpp"
#include "config.h"

void VoltageSensor::begin(uint8_t pin, float divider_ratio) {
    _pin = pin;
    _divider_ratio = divider_ratio;
    pinMode(_pin, INPUT);
}

float VoltageSensor::readVoltage() {
    // 複数回サンプリングして平均を取り、ADCノイズの影響を減らす
    uint32_t sum = 0;
    for (int i = 0; i < SAMPLE_NUM; i++) {
        sum += analogRead(_pin);
    }
    float raw_avg = (float)sum / SAMPLE_NUM;

    float voltage = raw_avg * (ADC_REF_VOLTAGE / ADC_RESOLUTION) * _divider_ratio;

    // 簡易ローパスフィルタ (EMA) でさらに安定化
    if (!_initialized) {
        _filtered = voltage;
        _initialized = true;
    } else {
        constexpr float ALPHA = 0.2f;
        _filtered = ALPHA * voltage + (1.0f - ALPHA) * _filtered;
    }

    return _filtered;
}
