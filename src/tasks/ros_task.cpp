#include "ros_task.hpp"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void rosTask(void *arg) {
    (void)arg;
    // TODO: micro-ROS初期化（現行ros_setup()相当を移管する）
    // TODO: agent未接続時は再接続待ちループ（制御タスクに影響させない）
    for (;;) {
        // TODO: executor spin、/cmd_vel→SystemContextキュー投入
        // TODO: SystemContextからテレメトリ取得→publish、/params→SystemContext反映
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
