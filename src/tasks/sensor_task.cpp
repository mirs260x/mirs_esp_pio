#include "sensor_task.hpp"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void sensorTask(void *arg) {
    (void)arg;
    // TODO: BMX055・VoltageSensor初期化を移管する
    for (;;) {
        // TODO: IMU・電圧ポーリング→SystemContextへ書込
        // TODO: analogRead平均化ループ・I2C読取はここに隔離する
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
