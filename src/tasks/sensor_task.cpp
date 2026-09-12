#include "sensor_task.hpp"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "hardware_config.hpp"
#include "VoltageSensor.hpp"
#include "BMX055.hpp"
#include "SystemContext.hpp"

using namespace mirs2605;

// 電圧読取は4回に1回（旧main.cppのPUBLISH_DIVIDER相当。変化が緩やかなため）
#define VOLTAGE_DIVIDER 4

static VoltageSensor voltage_sensor_1, voltage_sensor_2;
static BMX055 bmx055;
static BMX055Data bmx_data;

void sensorTask(void *arg) {
    (void)arg;

    // 電圧センサー初期化
    voltage_sensor_1.begin(PIN_BATT_1, VOLTAGE_DIVIDER_RATIO);
    voltage_sensor_2.begin(PIN_BATT_2, VOLTAGE_DIVIDER_RATIO);

    // BMX055 IMU初期化 (I2C_NUM_0, 400kHz, timeout 10ms)
    bmx055.begin(PIN_IMU_SDA, PIN_IMU_SCL, I2C_NUM_0, 400000, 10);

    float last_v1 = 0.0f, last_v2 = 0.0f;
    uint8_t div_cnt = 0;

    for (;;) {
        SharedSensor s{};

        // IMU読取（前回値を保持し、更新成功時のみフラグを立てる）
        if (bmx055.isInitialized() && bmx055.update(bmx_data)) {
            s.imu_ok = bmx055.isAccelOk() || bmx055.isGyroOk();
            s.mag_ok = bmx055.isMagOk();
            s.ax = bmx_data.ax;
            s.ay = bmx_data.ay;
            s.az = bmx_data.az;
            s.gx = bmx_data.gx;
            s.gy = bmx_data.gy;
            s.gz = bmx_data.gz;
            s.mx = bmx_data.mx;
            s.my = bmx_data.my;
            s.mz = bmx_data.mz;
        }

        // 電圧読取は4回に1回。間は前回値を保持する
        if (++div_cnt >= VOLTAGE_DIVIDER) {
            div_cnt = 0;
            last_v1 = voltage_sensor_1.readVoltage();
            last_v2 = voltage_sensor_2.readVoltage();
        }
        s.voltage_1 = last_v1;
        s.voltage_2 = last_v2;

        g_sys.setSensor(s);
        vTaskDelay(pdMS_TO_TICKS(TIMER_INTERVAL_MS));
    }
}
