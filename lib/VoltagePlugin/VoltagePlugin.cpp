#include "VoltagePlugin.hpp"
#include "VoltageSensor.hpp"

VoltagePlugin::VoltagePlugin(VoltageSensor &sensor, uint8_t channel, uint8_t pin,
                             float divider_ratio, float adc_ref_voltage, float adc_resolution)
    : sensor_(sensor)
    , channel_(channel)
    , pin_(pin)
    , divider_ratio_(divider_ratio)
    , adc_ref_voltage_(adc_ref_voltage)
    , adc_resolution_(adc_resolution)
{
}

bool VoltagePlugin::begin() {
    sensor_.begin(pin_, divider_ratio_, adc_ref_voltage_, adc_resolution_);
    return true;
}

void VoltagePlugin::update(SharedSensor &snapshot) {
    const float voltage = sensor_.readVoltage();
    if (channel_ == 1) {
        snapshot.voltage_1 = voltage;
    } else {
        snapshot.voltage_2 = voltage;
    }
}
