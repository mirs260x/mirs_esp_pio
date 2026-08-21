/*
 * mirs_esp_pio - micro-ROS ESP32 メインプログラム
 *
 * トピック:
 *   Subscribe: /cmd_vel  (geometry_msgs/Twist)
 *              /params   (mirs_msgs/BasicParam)
 *   Publish:  /encoder   (std_msgs/Int32MultiArray)  - エンコーダーカウント
 *             /vel       (std_msgs/Float64MultiArray) - 現在速度 [m/s]
 *             /vlt       (std_msgs/Float64MultiArray) - バッテリー電圧 [V]
 */

#include <micro_ros_platformio.h>
#include <Arduino.h>
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
#include "RcReceiver.hpp"
#include "VoltageSensor.hpp"
#include "SafetyEstop.hpp"
#include "BMX055.hpp"

// ============================================================
// 設定
// ============================================================
#define ROS_DOMAIN_ID             90
#define WATCHDOG_TIMEOUT          1000    // [ms] cmd_vel 無受信でWatchdog発火
#define TIMER_INTERVAL_MS         15      // [ms] 制御・パブリッシュ周期
#define PUBLISH_DIVIDER           4       // テレメトリはこの回数に1回パブリッシュ
#define ENABLE_VOLTAGE_CUTOFF     0       // 電圧カットオフ有効化 (配線確認後 1 に変更)
#define VOLTAGE_CUTOFF_THRESHOLD  20.0f   // [V]

// ============================================================
// PIDゲイン・車体パラメータ (/params トピックで動的更新可能)
// ============================================================
double RKP = 80.0, RKI = 30.0, RKD = 8.0;
double LKP = 80.0, LKI = 30.0, LKD = 8.0;
double WHEEL_RADIUS = 0.04;   // [m]
double WHEEL_BASE   = 0.38;   // [m]

// ============================================================
// グローバル変数
// ============================================================
volatile int32_t count_l = 0, count_r = 0;   // エンコーダーカウント (割り込みから更新)
double r_vel = 0, l_vel = 0;                  // 現在速度 [m/s]
double r_vel_cmd = 0, l_vel_cmd = 0;          // 速度指令 [m/s]
float  linear_x = 0, angular_z = 0;           // /cmd_vel から受信した速度指令
int32_t prev_count_l = 0, prev_count_r = 0;
float r_err_sum = 0, l_err_sum = 0;           // PID 積分項
float prev_r_err = 0, prev_l_err = 0;         // PID 微分項

uint32_t lastCalledAt = 0;                    // ウォッチドッグ用タイムスタンプ

enum ControlMode : uint8_t { MODE_MANUAL, MODE_ROS2 };
volatile ControlMode control_mode = MODE_MANUAL;

RcReceiver    rc_receiver;
VoltageSensor voltage_sensor_1, voltage_sensor_2;
SafetyEstop   safety_estop;
BMX055        bmx055;
BMX055Data    bmx_data;

// ============================================================
// micro-ROS オブジェクト
// ============================================================
rcl_allocator_t allocator;
rclc_support_t  support;
rcl_node_t      node;
rclc_executor_t executor;
rcl_timer_t     timer;

rcl_publisher_t    enc_pub, vel_pub, vlt_pub, rc_debug_pub, imu_pub, mag_pub;
rcl_subscription_t cmd_vel_sub, param_sub;

std_msgs__msg__Int32MultiArray   enc_msg;
std_msgs__msg__Float64MultiArray vel_msg, vlt_msg, rc_debug_msg;
sensor_msgs__msg__Imu            imu_msg;
sensor_msgs__msg__MagneticField  mag_msg;
geometry_msgs__msg__Twist        cmd_vel_msg;
mirs_msgs__msg__BasicParam       param_msg;

// ============================================================
// エンコーダー割り込みハンドラ
// ============================================================
static void IRAM_ATTR enc_change_l() {
    // 左エンコーダの方向を反転
    if (digitalRead(PIN_ENC_A_L) == digitalRead(PIN_ENC_B_L)) count_l = count_l + 1;
    else count_l = count_l - 1;
}
static void IRAM_ATTR enc_change_r() {
    if (digitalRead(PIN_ENC_A_R) == digitalRead(PIN_ENC_B_R)) count_r = count_r - 1;
    else count_r = count_r + 1;
}

// ============================================================
// 速度計算
// ============================================================
static void calculate_vel() {
    // 割り込みから更新される volatile カウンタを一度だけアトミックにコピーする。
    // count_l を2回読むと、1回目と2回目の間に割り込みが入り dl と
    // prev_count_l が食い違うため、ここでスナップショットを取る。
    portDISABLE_INTERRUPTS();
    const int32_t snap_l = count_l;
    const int32_t snap_r = count_r;
    portENABLE_INTERRUPTS();

    const int32_t dl = snap_l - prev_count_l;
    const int32_t dr = snap_r - prev_count_r;
    prev_count_l = snap_l;
    prev_count_r = snap_r;

    constexpr double dt = TIMER_INTERVAL_MS * 0.001;
    l_vel = (dl / COUNTS_PER_REV) * 2.0 * PI * WHEEL_RADIUS / dt;
    r_vel = (dr / COUNTS_PER_REV) * 2.0 * PI * WHEEL_RADIUS / dt;
}

// ============================================================
// PID 制御 + モーター出力
// ============================================================
static void pid_control() {
    /* E-Stop 一旦無効化
    if (g_estop_active) {
        ledcWrite(PIN_PWM_R, 0);
        ledcWrite(PIN_PWM_L, 0);
        r_err_sum = l_err_sum = 0;
        return;
    }
    */

    calculate_vel();

    float r_err = r_vel_cmd - r_vel;
    float l_err = l_vel_cmd - l_vel;
    r_err_sum += r_err;
    l_err_sum += l_err;

    double r_pwm = RKP * r_err + RKI * r_err_sum + RKD * (r_err - prev_r_err);
    double l_pwm = LKP * l_err + LKI * l_err_sum + LKD * (l_err - prev_l_err);
    prev_r_err = r_err;
    prev_l_err = l_err;

    r_pwm = constrain(r_pwm, -255.0, 255.0);
    l_pwm = constrain(l_pwm, -255.0, 255.0);

    // 速度指令ゼロ時はPWMと積分項をリセットする。
    // DIR ピンへの書き込みより先に行うことで方向ピンが不定にならないようにする。
    if (r_vel_cmd == 0) {
        r_pwm = 0;
        r_err_sum = 0;
        prev_r_err = 0;
    }
    if (l_vel_cmd == 0) {
        l_pwm = 0;
        l_err_sum = 0;
        prev_l_err = 0;
    }

    digitalWrite(PIN_DIR_R, r_pwm >= 0 ? LOW  : HIGH);
    digitalWrite(PIN_DIR_L, l_pwm >= 0 ? HIGH : LOW);

    ledcWrite(PIN_PWM_R, (uint8_t)abs(r_pwm));
    ledcWrite(PIN_PWM_L, (uint8_t)abs(l_pwm));
}

// ============================================================
// RC 入力から速度指令を更新
// ============================================================
static void update_rc_command() {
    const bool valid =
        rc_receiver.isSignalValid(CH_LEFT,    RC_SIGNAL_TIMEOUT_MS) &&
        rc_receiver.isSignalValid(CH_MODE_SW, RC_SIGNAL_TIMEOUT_MS) &&
        rc_receiver.isSignalValid(CH_RIGHT,   RC_SIGNAL_TIMEOUT_MS);

    if (!valid) {
        r_vel_cmd = l_vel_cmd = 0;
        return;
    }

    // スイッチの立ち上がりエッジ (0 -> 1) でモードを反転トグル (MANUAL <-> ROS2)
    static bool prev_sw_high = false;
    bool current_sw_high = (RcReceiver::pulseToNormalized(rc_receiver.getPulseWidth(CH_MODE_SW)) > 0.2f);

    if (current_sw_high && !prev_sw_high) {
        // 0 -> 1 に切り替わった瞬間に現在のモードを反転
        control_mode = (control_mode == MODE_MANUAL) ? MODE_ROS2 : MODE_MANUAL;
    }
    prev_sw_high = current_sw_high;

    if (control_mode == MODE_ROS2) {
        r_vel_cmd = linear_x + WHEEL_BASE / 2.0 * angular_z;
        l_vel_cmd = linear_x - WHEEL_BASE / 2.0 * angular_z;
        return;
    }

    // 手動モード: 左右スティックで左右輪を個別操作
    float l_norm = RcReceiver::pulseToNormalized(rc_receiver.getPulseWidth(CH_LEFT));
    float r_norm = RcReceiver::pulseToNormalized(rc_receiver.getPulseWidth(CH_RIGHT));
    l_vel_cmd = l_norm * MAX_LINEAR_SPEED;
    r_vel_cmd = r_norm * MAX_LINEAR_SPEED;
}

// ============================================================
// micro-ROS コールバック
// ============================================================
void cmd_vel_callback(const void *msgin) {
    const auto *msg = (const geometry_msgs__msg__Twist *)msgin;
    linear_x  = msg->linear.x;
    angular_z = msg->angular.z;
    lastCalledAt = millis();
}

void param_callback(const void *msgin) {
    const auto *p = (const mirs_msgs__msg__BasicParam *)msgin;
    WHEEL_RADIUS = p->wheel_radius;
    WHEEL_BASE   = p->wheel_base;
    RKP = p->rkp; RKI = p->rki; RKD = p->rkd;
    LKP = p->lkp; LKI = p->lki; LKD = p->lkd;
}

void timer_callback(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/) {
    update_rc_command();

    // ROS2 モード時のウォッチドッグ
    if (control_mode == MODE_ROS2 && (millis() - lastCalledAt) > WATCHDOG_TIMEOUT) {
        r_vel_cmd = l_vel_cmd = 0;
    }

    pid_control();

    // ----------------------------------------------------------------
    // IMU (BMX055) は毎周期 (15ms ≒ 67Hz) パブリッシュする。
    // タイムスタンプは micro-ROS エポック時刻 (ns) を使用する。
    // ----------------------------------------------------------------
    if (bmx055.isInitialized() && bmx055.update(bmx_data)) {
        // 現在時刻を取得 (micro-ROS エージェントと時刻同期済みの場合)
        int64_t now_ns = (int64_t)rmw_uros_epoch_nanos();
        int32_t now_sec  = (int32_t)(now_ns / 1000000000LL);
        uint32_t now_nsec = (uint32_t)(now_ns % 1000000000LL);

        if (bmx055.isAccelOk() || bmx055.isGyroOk()) {
            imu_msg.header.stamp.sec     = now_sec;
            imu_msg.header.stamp.nanosec = now_nsec;

            imu_msg.linear_acceleration.x = bmx_data.ax;
            imu_msg.linear_acceleration.y = bmx_data.ay;
            imu_msg.linear_acceleration.z = bmx_data.az;

            imu_msg.angular_velocity.x = bmx_data.gx;
            imu_msg.angular_velocity.y = bmx_data.gy;
            imu_msg.angular_velocity.z = bmx_data.gz;

            (void)rcl_publish(&imu_pub, &imu_msg, NULL);
        }

        if (bmx055.isMagOk()) {
            mag_msg.header.stamp.sec     = now_sec;
            mag_msg.header.stamp.nanosec = now_nsec;

            mag_msg.magnetic_field.x = bmx_data.mx * 1e-6f; // uT -> Tesla
            mag_msg.magnetic_field.y = bmx_data.my * 1e-6f;
            mag_msg.magnetic_field.z = bmx_data.mz * 1e-6f;

            (void)rcl_publish(&mag_pub, &mag_msg, NULL);
        }
    }

    // ----------------------------------------------------------------
    // テレメトリ (encoder / vel / vlt / rc_debug) は PUBLISH_DIVIDER 回に 1 回
    // ----------------------------------------------------------------
    enc_msg.data.data[0] = count_l;
    enc_msg.data.data[1] = count_r;
    vel_msg.data.data[0] = l_vel;
    vel_msg.data.data[1] = r_vel;

    static uint8_t div_cnt = 0;
    if (++div_cnt >= PUBLISH_DIVIDER) {
        div_cnt = 0;

        float v1 = voltage_sensor_1.readVoltage();
        float v2 = voltage_sensor_2.readVoltage();

#if ENABLE_VOLTAGE_CUTOFF
        if (v1 < VOLTAGE_CUTOFF_THRESHOLD || v2 < VOLTAGE_CUTOFF_THRESHOLD) {
            SafetyEstop::trigger();
        }
#endif

        vlt_msg.data.data[0] = v1;
        vlt_msg.data.data[1] = v2;

        // RC デバッグ情報: [0:ChLeft_us, 1:ChMode_us, 2:ChRight_us, 3:l_vel_cmd, 4:r_vel_cmd, 5:Mode(0:Manual,1:ROS2)]
        rc_debug_msg.data.data[0] = (double)rc_receiver.getPulseWidth(CH_LEFT);
        rc_debug_msg.data.data[1] = (double)rc_receiver.getPulseWidth(CH_MODE_SW);
        rc_debug_msg.data.data[2] = (double)rc_receiver.getPulseWidth(CH_RIGHT);
        rc_debug_msg.data.data[3] = l_vel_cmd;
        rc_debug_msg.data.data[4] = r_vel_cmd;
        rc_debug_msg.data.data[5] = (control_mode == MODE_ROS2) ? 1.0 : 0.0;

        (void)rcl_publish(&enc_pub, &enc_msg, NULL);
        (void)rcl_publish(&vel_pub, &vel_msg, NULL);
        (void)rcl_publish(&vlt_pub, &vlt_msg, NULL);
        (void)rcl_publish(&rc_debug_pub, &rc_debug_msg, NULL);
    }
}

// ============================================================
// micro-ROS セットアップ
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
    rclc_publisher_init_default(&enc_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/encoder");
    rclc_publisher_init_default(&vel_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "/vel");
    rclc_publisher_init_default(&vlt_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "/vlt");
    rclc_publisher_init_default(&rc_debug_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "/rc_debug");
    rclc_publisher_init_default(&imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data_raw");
    rclc_publisher_init_default(&mag_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), "/imu/mag");

    // サブスクライバー
    rclc_subscription_init_default(&cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
    rclc_subscription_init_default(&param_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(mirs_msgs, msg, BasicParam), "/params");

    // タイマー
    rclc_timer_init_default2(&timer, &support,
        RCL_MS_TO_NS(TIMER_INTERVAL_MS), timer_callback, true);

    // エグゼキューター: subscriber×2 + timer×1 = 3 ハンドル
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &param_sub,   &param_msg,   &param_callback,   ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);
}

// ============================================================
// メッセージバッファ確保
// ============================================================
static void alloc_messages() {
    enc_msg.data.capacity = 2; enc_msg.data.size = 2;
    enc_msg.data.data = (int32_t *)malloc(2 * sizeof(int32_t));
    enc_msg.data.data[0] = enc_msg.data.data[1] = 0;

    vel_msg.data.capacity = 2; vel_msg.data.size = 2;
    vel_msg.data.data = (double *)malloc(2 * sizeof(double));
    vel_msg.data.data[0] = vel_msg.data.data[1] = 0.0;

    vlt_msg.data.capacity = 2; vlt_msg.data.size = 2;
    vlt_msg.data.data = (double *)malloc(2 * sizeof(double));
    vlt_msg.data.data[0] = vlt_msg.data.data[1] = 0.0;

    rc_debug_msg.data.capacity = 6; rc_debug_msg.data.size = 6;
    rc_debug_msg.data.data = (double *)malloc(6 * sizeof(double));
    for (int i = 0; i < 6; i++) rc_debug_msg.data.data[i] = 0.0;

    imu_msg.header.frame_id.data = (char *)"imu_link";
    imu_msg.header.frame_id.size = strlen("imu_link");
    imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

    // orientation is not estimated on the sensor → mark as unknown (-1 in [0])
    // angular_velocity and linear_acceleration: diagonal 0.01 (rough estimate)
    imu_msg.orientation_covariance[0]          = -1.0;
    imu_msg.angular_velocity_covariance[0]     =  0.01;
    imu_msg.angular_velocity_covariance[4]     =  0.01;
    imu_msg.angular_velocity_covariance[8]     =  0.01;
    imu_msg.linear_acceleration_covariance[0]  =  0.01;
    imu_msg.linear_acceleration_covariance[4]  =  0.01;
    imu_msg.linear_acceleration_covariance[8]  =  0.01;

    mag_msg.header.frame_id.data = (char *)"imu_link";
    mag_msg.header.frame_id.size = strlen("imu_link");
    mag_msg.header.frame_id.capacity = mag_msg.header.frame_id.size + 1;

    // magnetic_field_covariance: diagonal 0.01 (rough estimate)
    mag_msg.magnetic_field_covariance[0] = 0.01;
    mag_msg.magnetic_field_covariance[4] = 0.01;
    mag_msg.magnetic_field_covariance[8] = 0.01;
}

// ============================================================
// setup / loop
// ============================================================
void setup() {
    // エンコーダー
    for (uint8_t pin : {PIN_ENC_A_L, PIN_ENC_B_L, PIN_ENC_A_R, PIN_ENC_B_R}) {
        pinMode(pin, INPUT_PULLUP);
    }
    attachInterrupt(PIN_ENC_A_L, enc_change_l, CHANGE);
    attachInterrupt(PIN_ENC_A_R, enc_change_r, CHANGE);

    // モーター PWM
    pinMode(PIN_DIR_R, OUTPUT);
    pinMode(PIN_DIR_L, OUTPUT);
    ledcAttach(PIN_PWM_R, 20000, 8);
    ledcAttach(PIN_PWM_L, 20000, 8);

    // 電圧センサー
    voltage_sensor_1.begin(PIN_BATT_1, VOLTAGE_DIVIDER_RATIO);
    voltage_sensor_2.begin(PIN_BATT_2, VOLTAGE_DIVIDER_RATIO);

    // RC レシーバー
    const uint8_t rc_pins[RC_NUM_CHANNELS] = {RC_LEFT_PIN, RC_MODE_SW_PIN, RC_RIGHT_PIN};
    rc_receiver.begin(rc_pins, RC_NUM_CHANNELS);

    // BMX055 IMU (esp-idf i2c_master: I2C_NUM_0, 400kHz, timeout 10ms)
    bmx055.begin(PIN_IMU_SDA, PIN_IMU_SCL, I2C_NUM_0, 400000, 10);

    // 非常停止 (回路実装前のため一旦無効化)
    // safety_estop.begin(ESTOP_PIN);

    alloc_messages();
    ros_setup();
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
}
