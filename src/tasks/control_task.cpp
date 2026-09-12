#include "control_task.hpp"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void controlTask(void *arg) {
    (void)arg;
    // TODO: 初期化（Encoder/DifferentialDrive/Odometry/MotorController/PID/RobotController）
    // TODO: main.cppのcontrol_loop()相当を移管する
    for (;;) {
        // TODO: 15ms周期で RC読取→調停→PID/直結→モータ出力
        // TODO: Encoder/Odometry更新→SystemContextへテレメトリ書込
        vTaskDelay(pdMS_TO_TICKS(15));
    }
}
