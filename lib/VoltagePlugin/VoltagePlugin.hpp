#pragma once
#include "SensorPlugin.hpp"

class VoltageSensor;

/** @brief 電圧センサプラグイン。担当ch（1/2）の電圧[V]を書き込む。 */
class VoltagePlugin : public ISensorPlugin {
public:
    /** @brief コンストラクタ。ボード固有値は全て注入する。
     *  @param sensor 実体（寿命は呼び出し側が保証すること）
     *  @param channel 1 → voltage_1、2 → voltage_2
     *  @param pin ADCピン
     *  @param divider_ratio 分圧比
     *  @param adc_ref_voltage ADC基準電圧 [V]
     *  @param adc_resolution ADC分解能
     */
    VoltagePlugin(VoltageSensor &sensor, uint8_t channel, uint8_t pin,
                  float divider_ratio, float adc_ref_voltage, float adc_resolution);

    const char *name() const override { return "voltage"; }
    bool begin() override;
    void update(SharedSensor &snapshot) override;

private:
    VoltageSensor &sensor_;
    uint8_t channel_;
    uint8_t pin_;
    float divider_ratio_;
    float adc_ref_voltage_;
    float adc_resolution_;
};
