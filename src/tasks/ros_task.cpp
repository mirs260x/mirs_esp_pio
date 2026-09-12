#include "ros_task.hpp"
#include <micro_ros_platformio.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <mirs_msgs/msg/basic_param.h>

#include "hardware_config.hpp"
#include "SafetyEstop.hpp"
#include "SystemContext.hpp"

using namespace mirs2605;

// ============================================================
// 設定（旧main.cppから移管。振る舞い同一）
// ============================================================
#define ROS_DOMAIN_ID 90
#define PUBLISH_DIVIDER 4  // テレメトリはこの回数に1回パブリッシュ
#define ENABLE_VOLTAGE_CUTOFF 0
#define VOLTAGE_CUTOFF_THRESHOLD 20.0f  // [V]

// ============================================================
// micro-ROS オブジェクト（旧main.cppグローバルを移管）
// ============================================================
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_timer_t timer;

static rcl_publisher_t enc_pub, vel_pub, vlt_pub, rc_debug_pub, imu_pub, mag_pub;
static rcl_subscription_t cmd_vel_sub, param_sub;

static std_msgs__msg__Int32MultiArray enc_msg;
static std_msgs__msg__Float64MultiArray vel_msg, vlt_msg, rc_debug_msg;
static sensor_msgs__msg__Imu imu_msg;
static sensor_msgs__msg__MagneticField mag_msg;
static geometry_msgs__msg__Twist cmd_vel_msg;
static mirs_msgs__msg__BasicParam param_msg;

// ============================================================
// micro-ROS コールバック
// ============================================================
static void cmd_vel_callback(const void *msgin) {
    const auto *msg = (const geometry_msgs__msg__Twist *)msgin;
    // 制御タスクへキュー投入する（直接触らない。ウォッチドッグは制御側で判定）
    RosVelocityCmd cmd;
    cmd.linear_x = msg->linear.x;
    cmd.angular_z = msg->angular.z;
    cmd.stamp_ms = millis();
    (void)g_sys.pushRosCmd(cmd);
}

static void param_callback(const void *msgin) {
    const auto *p = (const mirs_msgs__msg__BasicParam *)msgin;
    // SystemContextへ書込む。制御タスクが毎周期適用する
    SharedParams sp;
    sp.wheel_radius = p->wheel_radius;
    sp.wheel_base = p->wheel_base;
    sp.rkp = p->rkp;
    sp.rki = p->rki;
    sp.rkd = p->rkd;
    sp.lkp = p->lkp;
    sp.lki = p->lki;
    sp.lkd = p->lkd;
    g_sys.setParams(sp);
}

static void timer_callback(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/) {
    // SystemContextからスナップショットを取得する（旧来の直接参照の代替）
    const SharedMotion motion = g_sys.getMotion();
    const SharedSensor sensor = g_sys.getSensor();

    // ----------------------------------------------------------------
    // IMU (BMX055) は毎周期 (15ms ≒ 67Hz) パブリッシュする。
    // タイムスタンプは micro-ROS エポック時刻 (ns) を使用する。
    // ----------------------------------------------------------------
    if (sensor.imu_ok || sensor.mag_ok) {
        int64_t now_ns = (int64_t)rmw_uros_epoch_nanos();
        int32_t now_sec = (int32_t)(now_ns / 1000000000LL);
        uint32_t now_nsec = (uint32_t)(now_ns % 1000000000LL);

        if (sensor.imu_ok) {
            imu_msg.header.stamp.sec = now_sec;
            imu_msg.header.stamp.nanosec = now_nsec;

            imu_msg.linear_acceleration.x = sensor.ax;
            imu_msg.linear_acceleration.y = sensor.ay;
            imu_msg.linear_acceleration.z = sensor.az;

            imu_msg.angular_velocity.x = sensor.gx;
            imu_msg.angular_velocity.y = sensor.gy;
            imu_msg.angular_velocity.z = sensor.gz;

            (void)rcl_publish(&imu_pub, &imu_msg, NULL);
        }

        if (sensor.mag_ok) {
            mag_msg.header.stamp.sec = now_sec;
            mag_msg.header.stamp.nanosec = now_nsec;

            mag_msg.magnetic_field.x = sensor.mx * 1e-6f;  // uT -> Tesla
            mag_msg.magnetic_field.y = sensor.my * 1e-6f;
            mag_msg.magnetic_field.z = sensor.mz * 1e-6f;

            (void)rcl_publish(&mag_pub, &mag_msg, NULL);
        }
    }

    // ----------------------------------------------------------------
    // テレメトリ (encoder / vel / vlt / rc_debug) は PUBLISH_DIVIDER 回に 1 回
    // ----------------------------------------------------------------
    enc_msg.data.data[0] = motion.count_l;
    enc_msg.data.data[1] = motion.count_r;
    vel_msg.data.data[0] = motion.vel_l;
    vel_msg.data.data[1] = motion.vel_r;

    static uint8_t div_cnt = 0;
    if (++div_cnt >= PUBLISH_DIVIDER) {
        div_cnt = 0;

        float v1 = sensor.voltage_1;
        float v2 = sensor.voltage_2;

#if ENABLE_VOLTAGE_CUTOFF
        if (v1 < VOLTAGE_CUTOFF_THRESHOLD || v2 < VOLTAGE_CUTOFF_THRESHOLD) {
            SafetyEstop::trigger();
        }
#endif

        vlt_msg.data.data[0] = v1;
        vlt_msg.data.data[1] = v2;

        // RC デバッグ情報: [0:ChLeft_us, 1:ChMode_us, 2:ChRight_us, 3:l_vel_cmd, 4:r_vel_cmd, 5:Mode(0:Manual,1:ROS2)]
        rc_debug_msg.data.data[0] = (double)motion.rc_pulse[0];
        rc_debug_msg.data.data[1] = (double)motion.rc_pulse[1];
        rc_debug_msg.data.data[2] = (double)motion.rc_pulse[2];
        rc_debug_msg.data.data[3] = motion.vel_cmd_l;
        rc_debug_msg.data.data[4] = motion.vel_cmd_r;
        rc_debug_msg.data.data[5] = (double)motion.ctrl_mode;

        (void)rcl_publish(&enc_pub, &enc_msg, NULL);
        (void)rcl_publish(&vel_pub, &vel_msg, NULL);
        (void)rcl_publish(&vlt_pub, &vlt_msg, NULL);
        (void)rcl_publish(&rc_debug_pub, &rc_debug_msg, NULL);
    }
}

// ============================================================
// micro-ROS セットアップ（旧main.cpp ros_setup()と同一）
// ============================================================
static void ros_setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);

    allocator = rcl_get_default_allocator();

    // ROS_DOMAIN_ID 設定 (Jazzy 以降)
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    (void)rcl_init_options_init(&init_options, allocator);
    (void)rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    rcl_node_options_t node_ops = rcl_node_get_default_options();
    rclc_node_init_with_options(&node, "ESP32_node", "", &support, &node_ops);

    // パブリッシャー
    rclc_publisher_init_default(&enc_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/encoder");
    rclc_publisher_init_default(&vel_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "/vel");
    rclc_publisher_init_default(&vlt_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "/vlt");
    rclc_publisher_init_default(&rc_debug_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "/rc_debug");
    rclc_publisher_init_default(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data_raw");
    rclc_publisher_init_default(&mag_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), "/imu/mag");

    // サブスクライバー
    rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
    rclc_subscription_init_default(&param_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(mirs_msgs, msg, BasicParam), "/params");

    // タイマー
    rclc_timer_init_default2(&timer, &support,
                             RCL_MS_TO_NS(TIMER_INTERVAL_MS), timer_callback, true);

    // エグゼキューター: subscriber×2 + timer×1 = 3 ハンドル
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &param_sub, &param_msg, &param_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);
}

// ============================================================
// メッセージバッファ確保（旧main.cpp alloc_messages()と同一）
// ============================================================
static void alloc_messages() {
    enc_msg.data.capacity = 2;
    enc_msg.data.size = 2;
    enc_msg.data.data = (int32_t *)malloc(2 * sizeof(int32_t));
    enc_msg.data.data[0] = enc_msg.data.data[1] = 0;

    vel_msg.data.capacity = 2;
    vel_msg.data.size = 2;
    vel_msg.data.data = (double *)malloc(2 * sizeof(double));
    vel_msg.data.data[0] = vel_msg.data.data[1] = 0.0;

    vlt_msg.data.capacity = 2;
    vlt_msg.data.size = 2;
    vlt_msg.data.data = (double *)malloc(2 * sizeof(double));
    vlt_msg.data.data[0] = vlt_msg.data.data[1] = 0.0;

    rc_debug_msg.data.capacity = 6;
    rc_debug_msg.data.size = 6;
    rc_debug_msg.data.data = (double *)malloc(6 * sizeof(double));
    for (int i = 0; i < 6; i++) rc_debug_msg.data.data[i] = 0.0;

    imu_msg.header.frame_id.data = (char *)"imu_link";
    imu_msg.header.frame_id.size = strlen("imu_link");
    imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

    // orientation is not estimated on the sensor → mark as unknown (-1 in [0])
    // angular_velocity and linear_acceleration: diagonal 0.01 (rough estimate)
    imu_msg.orientation_covariance[0] = -1.0;
    imu_msg.angular_velocity_covariance[0] = 0.01;
    imu_msg.angular_velocity_covariance[4] = 0.01;
    imu_msg.angular_velocity_covariance[8] = 0.01;
    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.linear_acceleration_covariance[4] = 0.01;
    imu_msg.linear_acceleration_covariance[8] = 0.01;

    mag_msg.header.frame_id.data = (char *)"imu_link";
    mag_msg.header.frame_id.size = strlen("imu_link");
    mag_msg.header.frame_id.capacity = mag_msg.header.frame_id.size + 1;

    // magnetic_field_covariance: diagonal 0.01 (rough estimate)
    mag_msg.magnetic_field_covariance[0] = 0.01;
    mag_msg.magnetic_field_covariance[4] = 0.01;
    mag_msg.magnetic_field_covariance[8] = 0.01;
}

void rosTask(void *arg) {
    (void)arg;
    alloc_messages();
    ros_setup();
    for (;;) {
        // micro-ROS通信処理（ノンブロッキング）
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
