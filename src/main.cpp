/*
 * mirs_esp_pio - micro-ROS ESP32 メインプログラム
 *
 * タスク構成（詳細は README.md の仕様章）:
 *   control task (Core0): RC読取→調停→PID/直結→モータ出力。ROS非依存で完結
 *   ros task     (Core1): micro-ROS通信・テレメトリ発行・パラメータ反映
 *   sensor task  (Core1): IMU・電圧ポーリング
 *
 * トピック:
 *   Subscribe: /cmd_vel  (geometry_msgs/Twist)
 *              /params   (mirs_msgs/BasicParam)
 *   Publish:  /encoder   (std_msgs/Int32MultiArray)  - エンコーダーカウント
 *             /vel       (std_msgs/Float64MultiArray) - 現在速度 [m/s]
 *             /vlt       (std_msgs/Float64MultiArray) - バッテリー電圧 [V]
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "SystemContext.hpp"
#include "tasks/control_task.hpp"
#include "tasks/ros_task.hpp"
#include "tasks/sensor_task.hpp"

void setup() {
    g_sys.begin();

    // 各タスクは自身のハード初期化を持つ。mainは生成のみ行う
    xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 10, nullptr, 0);
    xTaskCreatePinnedToCore(rosTask, "ros", 12288, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(sensorTask, "sensor", 4096, nullptr, 3, nullptr, 1);
}

void loop() {
    // 全処理はタスク側。Arduinoタスクはウォッチドッグ用に待機のみ
    vTaskDelay(pdMS_TO_TICKS(1000));
}
