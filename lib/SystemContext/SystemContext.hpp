#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// タスク間共有の置き場所。tasksとlibの結合点。
// ROSの型（Twist等）は持ち込まない。恩赦対象外の新規層。

// ROS速度指令（ros_task → control）。キューで受渡す
struct RosVelocityCmd {
    float linear_x = 0.0f;
    float angular_z = 0.0f;
    uint32_t stamp_ms = 0;
};

// 車体・PIDパラメータ（/params受信→各タスク参照）。mutex保護
struct SharedParams {
    double wheel_radius = 0.04;
    double wheel_base = 0.38;
    double rkp = 80.0, rki = 30.0, rkd = 8.0;
    double lkp = 80.0, lki = 30.0, lkd = 8.0;
};

// テレメトリスナップショット（各タスク書込→ros_task発行）。mutex保護
struct SharedTelemetry {
    int32_t count_l = 0;
    int32_t count_r = 0;
    float odom_x = 0.0f;
    float odom_y = 0.0f;
    float odom_theta = 0.0f;
    float vel_l = 0.0f;
    float vel_r = 0.0f;
    float voltage_1 = 0.0f;
    float voltage_2 = 0.0f;
    bool imu_valid = false;
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float mx = 0.0f, my = 0.0f, mz = 0.0f;
};

class SystemContext {
public:
    void begin();  // キュー・mutex生成

    // ROS指令キュー（非ブロッキング）
    bool pushRosCmd(const RosVelocityCmd &cmd);
    bool popRosCmd(RosVelocityCmd &cmd);

    // パラメータ・テレメトリ（mutex保護コピー）
    void setParams(const SharedParams &p);
    SharedParams getParams();
    void setTelemetry(const SharedTelemetry &t);
    SharedTelemetry getTelemetry();

private:
    QueueHandle_t ros_cmd_queue_ = nullptr;
    SemaphoreHandle_t param_mutex_ = nullptr;
    SemaphoreHandle_t telemetry_mutex_ = nullptr;
    SharedParams params_;
    SharedTelemetry telemetry_;
};

extern SystemContext g_sys;
