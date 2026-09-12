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
#include "VelocityCalculator.hpp"
#include "PIDController.hpp"
#include "RobotController.hpp"
#include "MotorController.hpp"

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
int32_t prev_count_l = 0, prev_count_r = 0;

RcReceiver      rc_receiver;
VoltageSensor   voltage_sensor_1, voltage_sensor_2;
SafetyEstop     safety_estop;
BMX055          bmx055;
BMX055Data      bmx_data;

// 制御ライブラリのインスタンス
VelocityCalculator vel_calc(COUNTS_PER_REV, WHEEL_RADIUS, TIMER_INTERVAL_MS * 0.001);
PIDController      pid_right(RKP, RKI, RKD);
PIDController      pid_left(LKP, LKI, LKD);
RobotController    robot_ctrl(rc_receiver, WHEEL_BASE, MAX_LINEAR_SPEED, WATCHDOG_TIMEOUT);
MotorController    motor_ctrl(PIN_DIR_L, PIN_PWM_L, PIN_DIR_R, PIN_PWM_R);

// 制御ループ実行フラグ（ハードウェアタイマーから設定）
volatile bool control_loop_flag = false;
portMUX_TYPE controlMux = portMUX_INITIALIZER_UNLOCKED;

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
	if (digitalRead(PIN_ENC_A_L) == digitalRead(PIN_ENC_B_L)) {
		count_l = count_l - 1;
	} else {
		count_l = count_l + 1;
	}
}
static void IRAM_ATTR enc_change_r() {
	if (digitalRead(PIN_ENC_A_R) == digitalRead(PIN_ENC_B_R)) {
		count_r = count_r + 1;
	} else {
		count_r = count_r - 1;
	}
}

// ============================================================
// ハードウェアタイマー割り込み（軽量フラグ設定のみ）
// ============================================================
hw_timer_t *control_timer = NULL;

void IRAM_ATTR onControlTimer() {
	// 割り込みハンドラ内では最小限の処理のみ：フラグを立てるだけ
	portENTER_CRITICAL_ISR(&controlMux);
	control_loop_flag = true;
	portEXIT_CRITICAL_ISR(&controlMux);
}

// ============================================================
// 制御ループ処理（loop()から呼び出される）
// ============================================================
static void control_loop() {
	// RC入力と制御モード更新
	robot_ctrl.update(CH_LEFT, CH_MODE_SW, CH_RIGHT, RC_SIGNAL_TIMEOUT_MS);

	portENTER_CRITICAL(&controlMux);
	const int32_t snap_l = count_l;
	const int32_t snap_r = count_r;
	portEXIT_CRITICAL(&controlMux);

	// 速度計算
	vel_calc.calculateBothWheels(snap_l, snap_r, prev_count_l, prev_count_r, l_vel, r_vel);

	// 速度指令を取得
	const double r_vel_cmd = robot_ctrl.getRightVelCmd();
	const double l_vel_cmd = robot_ctrl.getLeftVelCmd();

	// PID制御計算
	double r_pwm = pid_right.compute(r_vel_cmd, r_vel);
	double l_pwm = pid_left.compute(l_vel_cmd, l_vel);

	// 速度指令ゼロ時はPWMと積分項をリセット
	if (r_vel_cmd == 0.0) {
		r_pwm = 0.0;
		pid_right.reset();
	}
	if (l_vel_cmd == 0.0) {
		l_pwm = 0.0;
		pid_left.reset();
	}

	// モーター出力
	motor_ctrl.setBothMotors(l_pwm, r_pwm);
}

// ============================================================
// micro-ROS コールバック
// ============================================================
void cmd_vel_callback(const void *msgin) {
	const auto *msg = (const geometry_msgs__msg__Twist *)msgin;
	// RobotControllerに速度指令を渡す（ウォッチドッグタイマーもリセット）
	robot_ctrl.updateRos2Command(msg->linear.x, msg->angular.z);
}

void param_callback(const void *msgin) {
	const auto *p = (const mirs_msgs__msg__BasicParam *)msgin;

	// 車輪パラメータをライブラリに反映
	vel_calc.setWheelRadius(p->wheel_radius);
	robot_ctrl.setWheelBase(p->wheel_base);

	// PIDゲインをライブラリに反映
	pid_right.setGains(p->rkp, p->rki, p->rkd);
	pid_left.setGains(p->lkp, p->lki, p->lkd);
}

void timer_callback(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/) {
	// この関数はmicro-ROSのパブリッシュのみを担当します。
	// 制御ループはloop()内で実行されるため、micro-ROSが落ちても制御は継続します。

	// 現在の速度を取得（スレッドセーフ）
	portENTER_CRITICAL(&controlMux);
	double current_l_vel = l_vel;
	double current_r_vel = r_vel;
	double l_vel_cmd = robot_ctrl.getLeftVelCmd();
	double r_vel_cmd = robot_ctrl.getRightVelCmd();
	portEXIT_CRITICAL(&controlMux);

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
	portENTER_CRITICAL(&controlMux);
	int32_t snap_count_l = count_l;
	int32_t snap_count_r = count_r;
	portEXIT_CRITICAL(&controlMux);

	enc_msg.data.data[0] = snap_count_l;
	enc_msg.data.data[1] = snap_count_r;
	vel_msg.data.data[0] = current_l_vel;
	vel_msg.data.data[1] = current_r_vel;

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
		rc_debug_msg.data.data[5] = (robot_ctrl.getControlMode() == RobotController::MODE_ROS2) ? 1.0 : 0.0;

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
	rclc_publisher_init_default(&enc_pub, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/encoder");
	rclc_publisher_init_default(&vel_pub, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "/vel");
	rclc_publisher_init_default(&vlt_pub, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "/vlt");
	rclc_publisher_init_default(&rc_debug_pub, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "/rc_debug");
	rclc_publisher_init_default(&imu_pub, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data_raw");
	rclc_publisher_init_default(&mag_pub, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), "/imu/mag");

	// サブスクライバー
	rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
	rclc_subscription_init_default(&param_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(mirs_msgs, msg, BasicParam), "/params");

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
	// エンコーダー初期化
	for (uint8_t pin : {PIN_ENC_A_L, PIN_ENC_B_L, PIN_ENC_A_R, PIN_ENC_B_R}) {
		pinMode(pin, INPUT_PULLUP);
	}
	attachInterrupt(PIN_ENC_A_L, enc_change_l, CHANGE);
	attachInterrupt(PIN_ENC_A_R, enc_change_r, CHANGE);

	// モーター制御初期化
	motor_ctrl.begin(20000, 8);  // 20kHz, 8bit分解能

	// 電圧センサー初期化
	voltage_sensor_1.begin(PIN_BATT_1, VOLTAGE_DIVIDER_RATIO);
	voltage_sensor_2.begin(PIN_BATT_2, VOLTAGE_DIVIDER_RATIO);

	// RC レシーバー初期化
	const uint8_t rc_pins[RC_NUM_CHANNELS] = {RC_LEFT_PIN, RC_MODE_SW_PIN, RC_RIGHT_PIN};
	rc_receiver.begin(rc_pins, RC_NUM_CHANNELS);

	// BMX055 IMU初期化 (I2C_NUM_0, 400kHz, timeout 10ms)
	bmx055.begin(PIN_IMU_SDA, PIN_IMU_SCL, I2C_NUM_0, 400000, 10);

	// 非常停止初期化 (回路実装前のため一旦無効化)
	// safety_estop.begin(ESTOP_PIN);

	// ハードウェアタイマー設定（micro-ROS非依存で動作）
	// TIMER_INTERVAL_MS = 15ms → 66.67Hz
	const uint32_t timer_freq_hz = 1000 / TIMER_INTERVAL_MS;
	control_timer = timerBegin(timer_freq_hz);
	timerAttachInterrupt(control_timer, &onControlTimer);
	timerAlarm(control_timer, 1000000 / timer_freq_hz, true, 0);

	// micro-ROS初期化
	alloc_messages();
	ros_setup();
}

void loop() {
	// micro-ROS通信処理（ノンブロッキング）
	rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

	// 制御ループフラグをチェック
	portENTER_CRITICAL(&controlMux);
	bool should_run = control_loop_flag;
	if (should_run) {
		control_loop_flag = false;  // フラグをクリア
	}
	portEXIT_CRITICAL(&controlMux);

	// フラグが立っていれば制御ループを実行
	if (should_run) {
		control_loop();
	}
}
