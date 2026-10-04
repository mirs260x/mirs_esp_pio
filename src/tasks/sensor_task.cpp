#include "sensor_task.hpp"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "hardware_config.hpp"
#include "VoltageSensor.hpp"
#include "BMX055.hpp"
#include "Registry.hpp"
#include "SensorPlugin.hpp"
#include "VoltagePlugin.hpp"
#include "ImuPlugin.hpp"
#include "SystemContext.hpp"

// 電圧は変化が緩やかなため4制御周期に1回（約60ms）読む
#define VOLTAGE_DIVIDER 4
#define MAX_PLUGINS 4

// ---- 合成ルート：追加・削除はこの登録リスト1行 ----
// ハード実体とプラグインの寿命はタスクと同一（static）。
// IMU不在時は begin() 失敗で無効化されるため、#if分離はしない。
static VoltageSensor voltage_sensor_1, voltage_sensor_2;
static BMX055 bmx055;
static VoltagePlugin voltage_plugin_1(voltage_sensor_1, 1, PIN_BATT_1,
                                      VOLTAGE_DIVIDER_RATIO, ADC_REF_VOLTAGE, ADC_RESOLUTION);
static VoltagePlugin voltage_plugin_2(voltage_sensor_2, 2, PIN_BATT_2,
                                      VOLTAGE_DIVIDER_RATIO, ADC_REF_VOLTAGE, ADC_RESOLUTION);
static ImuPlugin imu_plugin(bmx055, PIN_IMU_SDA, PIN_IMU_SCL);
static Registry<ISensorPlugin, SharedSensor, MAX_PLUGINS> registry;

void sensorTask(void *arg) {
    (void)arg;

    (void)registry.add(&voltage_plugin_1, VOLTAGE_DIVIDER);
    (void)registry.add(&voltage_plugin_2, VOLTAGE_DIVIDER);
    (void)registry.add(&imu_plugin, 1);

    for (size_t i = 0; i < registry.size(); ++i) {
        if (!registry.beginPlugin(i)) {
            Serial.printf("[sensor] %s unavailable, disabled\n", registry.name(i));
        }
    }

    // 蓄積スナップショット。間引き中の電圧は前回値を保持する
    static SharedSensor snapshot{};

    for (;;) {
        registry.updateAll(snapshot);
        g_sys.setSensor(snapshot);
        vTaskDelay(pdMS_TO_TICKS(TIMER_INTERVAL_MS));
    }
}
