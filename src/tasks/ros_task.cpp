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
#include <std_msgs/msg/empty.h>
#include <mirs_msgs/msg/basic_param.h>
#include <cmath>

#include "../publishers/imu_publisher.hpp"
#include "../publishers/odom_publisher.hpp"
#include "../publishers/publisher_plugin.hpp"
#include "../publishers/telemetry_publisher.hpp"
#include "hardware_config.hpp"
#include "SystemContext.hpp"
#include "control_task.hpp"

// ============================================================
// 通信タスク設定
// ============================================================
#define ROS_DOMAIN_ID 90

// /params受付範囲（範囲外・NaNは車体を不安定化させるため棄却する）
#define PARAM_RADIUS_MAX 0.5   // [m]
#define PARAM_BASE_MAX 2.0     // [m]
#define PARAM_GAIN_ABS_MAX 1000.0

// ============================================================
// micro-ROS オブジェクト
// ============================================================
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_timer_t timer;

static rcl_subscription_t cmd_vel_sub, param_sub, reset_odom_sub;

static geometry_msgs__msg__Twist cmd_vel_msg;
static mirs_msgs__msg__BasicParam param_msg;
static std_msgs__msg__Empty reset_odom_msg;

// ---- 合成ルート：発行トピックの追加・削除はこの登録リスト1行 ----
// データ不在時の無発行は各プラグインが担当するため、#if分離はしない。
static ImuPublisher imu_pub_plugin;
static OdomPublisher odom_pub_plugin;
static TelemetryPublisher telemetry_pub_plugin;
static IPublisherPlugin *publishers[] = {
    &imu_pub_plugin,
    &odom_pub_plugin,
    &telemetry_pub_plugin,
};

// /params最終受信時刻。parameter_publisherは500ms周期のため、
// agent到達可なのに3s途絶えたら陳腐セッションとみなして再setupする。
static uint32_t last_param_ms = 0;
static bool params_seen = false;

// micro-ROS コールバック
static void cmd_vel_callback(const void *msgin) {
    const auto *msg = (const geometry_msgs__msg__Twist *)msgin;
    // 制御タスクへmailbox投入する（直接触らない。ウォッチドッグは制御側で判定）
    RosVelocityCmd cmd;
    cmd.linear_x = msg->linear.x;
    cmd.angular_z = msg->angular.z;
    cmd.stamp_ms = millis();
    (void)g_sys.pushRosCmd(cmd);
}

static bool paramsInRange(const mirs_msgs__msg__BasicParam *p) {
    if (!std::isfinite(p->wheel_radius) || p->wheel_radius <= 0.0 ||
        p->wheel_radius > PARAM_RADIUS_MAX) {
        return false;
    }
    if (!std::isfinite(p->wheel_base) || p->wheel_base <= 0.0 ||
        p->wheel_base > PARAM_BASE_MAX) {
        return false;
    }
    const double gains[6] = {p->rkp, p->rki, p->rkd, p->lkp, p->lki, p->lkd};
    for (double g : gains) {
        if (!std::isfinite(g) || std::fabs(g) > PARAM_GAIN_ABS_MAX) {
            return false;
        }
    }
    return true;
}

static void param_callback(const void *msgin) {
    const auto *p = (const mirs_msgs__msg__BasicParam *)msgin;
    last_param_ms = millis();
    params_seen = true;
    if (!paramsInRange(p)) {
        Serial.println("[ros] /params rejected (out of range or NaN)");
        return;
    }
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

// 原点リセット要求（停止中に呼ぶこと）。制御周期内でodom x/y/thetaをゼロ化する。
static void reset_odom_callback(const void * /*msgin*/) {
    requestOdometryReset();
}

static void timer_callback(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/) {
    const SharedMotion motion = g_sys.getMotion();
    const SharedSensor sensor = g_sys.getSensor();

    const int64_t now_ns = (int64_t)rmw_uros_epoch_nanos();
    const int32_t now_sec = (int32_t)(now_ns / 1000000000LL);
    const uint32_t now_nsec = (uint32_t)(now_ns % 1000000000LL);

    for (IPublisherPlugin *pub : publishers) {
        pub->publish(motion, sensor, now_sec, now_nsec);
    }
}

// micro-ROS セットアップ。成功時true、失敗時はrosTask側でteardown→リトライする
#define ROS_CHECK(fn)                                     \
    do {                                                  \
        rcl_ret_t _rc = (fn);                             \
        if (_rc != RCL_RET_OK) {                          \
            Serial.printf("[ros] setup failed: %s:%d rc=%d\n", __func__, __LINE__, (int)_rc); \
            return false;                                 \
        }                                                 \
    } while (0)

static bool ros_setup() {
    allocator = rcl_get_default_allocator();

    // ROS_DOMAIN_ID 設定 (Jazzy 以降)
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    ROS_CHECK(rcl_init_options_init(&init_options, allocator));
    rcl_ret_t domain_rc = rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);
    if (domain_rc != RCL_RET_OK) {
        Serial.printf("[ros] domain_id failed rc=%d\n", (int)domain_rc);
        ignoreResult(rcl_init_options_fini(&init_options));
        return false;
    }
    rcl_ret_t support_rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    ignoreResult(rcl_init_options_fini(&init_options));
    if (support_rc != RCL_RET_OK) {
        Serial.printf("[ros] support init failed rc=%d (agent不在の可能性)\n", (int)support_rc);
        return false;
    }

    rcl_node_options_t node_ops = rcl_node_get_default_options();
    ROS_CHECK(rclc_node_init_with_options(&node, "ESP32_node", "", &support, &node_ops));

    // パブリッシャー（登録リスト順。失敗時は確保分を戻して全体失敗）
    for (IPublisherPlugin *pub : publishers) {
        if (!pub->advertise(&node)) {
            Serial.printf("[ros] advertise failed: %s\n", pub->name());
            for (IPublisherPlugin *rel : publishers) {
                rel->release(&node);
            }
            return false;
        }
    }

    // サブスクライバー
    ROS_CHECK(rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));
    ROS_CHECK(rclc_subscription_init_default(&param_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(mirs_msgs, msg, BasicParam), "/params"));
    ROS_CHECK(rclc_subscription_init_default(&reset_odom_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty), "/reset_odometry"));

    // タイマー
    ROS_CHECK(rclc_timer_init_default2(&timer, &support,
                             RCL_MS_TO_NS(TIMER_INTERVAL_MS), timer_callback, true));

    // エグゼキューター: subscriber×3 + timer×1 = 4 ハンドル
    ROS_CHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    ROS_CHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
    ROS_CHECK(rclc_executor_add_subscription(&executor, &param_sub, &param_msg, &param_callback, ON_NEW_DATA));
    ROS_CHECK(rclc_executor_add_subscription(&executor, &reset_odom_sub, &reset_odom_msg, &reset_odom_callback, ON_NEW_DATA));
    ROS_CHECK(rclc_executor_add_timer(&executor, &timer));
    return true;
}

// 部分初期化済みリソースの後始末。未初期化へのfiniはエラーを無視する
static void ros_teardown() {
    ignoreResult(rclc_executor_fini(&executor));
    ignoreResult(rcl_timer_fini(&timer));
    ignoreResult(rcl_subscription_fini(&cmd_vel_sub, &node));
    ignoreResult(rcl_subscription_fini(&param_sub, &node));
    ignoreResult(rcl_subscription_fini(&reset_odom_sub, &node));
    for (IPublisherPlugin *pub : publishers) {
        pub->release(&node);
    }
    ignoreResult(rcl_node_fini(&node));
    ignoreResult(rclc_support_fini(&support));
}

void rosTask(void *arg) {
    (void)arg;
    Serial.begin(115200);
    Serial.printf("[fw] mirs_esp_pio %s\n", FW_VERSION);
    set_microros_serial_transports(Serial);

    // agent不在でもcontrol/sensorは継続。rosタスクのみここで待機・リトライする
    // （advertise失敗時もteardown→リトライする。control/sensorは継続する）
    for (;;) {
        // --- agent待ち (他タスクは継続中) ---
        while (rmw_uros_ping_agent(500, 2) != RMW_RET_OK) {
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        if (!ros_setup()) {
            ros_teardown();
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        // 時刻同期はsupport初期化後（＝setup後）に行う。初期化前のsyncは
        // 効かず、未同期stampでのpublishはEKFを不可逆汚染するため出さない。
        bool synced = false;
        for (int i = 0; i < 5 && !synced; ++i) {
            synced = (rmw_uros_sync_session(1000) == RMW_RET_OK);
        }
        if (!synced) {
            Serial.println("[ros] time sync failed, retrying setup");
            ros_teardown();
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        last_param_ms = millis();  // 猶予起点（直後の誤検出フラップ防止）
        // --- 接続中ループ。切断検出で抜けて再接続へ ---
        bool time_synced = true;  // setup直後に同期済み。再同期の遷移検出用。
        uint32_t last_ping_ms = millis();
        uint8_t ping_fail = 0;
        bool session_broken = false;
        while (!session_broken) {
            // micro-ROS通信処理（ノンブロッキング）
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
            // agent再起動後の陳腐セッション対策：pingは通るが購読が届かない状態を
            // 500ms周期の/params途絶（3s）で検出して作り直す。一度も受信前は判定しない。
            if (params_seen && (millis() - last_param_ms > 3000)) {
                Serial.println("[ros] params stalled, re-syncing session");
                session_broken = true;
                break;
            }
            if ((millis() - last_ping_ms) >= 2000) {
                last_ping_ms = millis();
                if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                    if (++ping_fail >= 3) {
                        Serial.println("[ros] agent lost, reconnecting");
                        session_broken = true;
                    }
                } else {
                    ping_fail = 0;
                    // 接続後の再同期（水晶ドリフト対策）。成功遷移時のみログする。
                    // NOTE: 未同期publishは行わない方針（setup直後に同期済みのため）。
                    if (rmw_uros_sync_session(100) == RMW_RET_OK && !time_synced) {
                        time_synced = true;
                        Serial.println("[ros] time synced");
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        ros_teardown();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
