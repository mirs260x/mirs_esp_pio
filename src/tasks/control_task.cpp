#include "control_task.hpp"
#include <Arduino.h>
#include <cmath>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "hardware_config.hpp"
#include "Encoder.hpp"
#include "DiffDrive.hpp"
#include "OdometryCalculator.hpp"
#include "RcReceiver.hpp"
#include "VelocityCalculator.hpp"
#include "PIDController.hpp"
#include "RobotController.hpp"
#include "MotorDriver.hpp"
#include "DiffMotors.hpp"
#include "SystemContext.hpp"

// ============================================================
// 責務分割：control_taskは配線と実行順序のみ持つ。
// 計算・調停・出力・共有は各層に委譲し、ステージ間の受渡しは
// 小さな値型（MotionSample/WheelCmd）で明示する。
//
// パイプライン（上→下にのみ依存）：
//   指令受付 → パラメータ適用 → 調停 → 状態推定 → 制御・出力 → テレメトリ
//   RobotController: MANUAL/ROS2調停・watchdog
//   VelocityCalculator/PIDController/OdometryCalculator: 計算層
//   DiffMotors/MotorDriver: 出力層（IF＋デバイス）
//   SystemContext: タスク間共有
// ============================================================
namespace {

// 設定値の唯一の出所はSharedParams既定値。ここでの二重定義はしない
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 1000;  // [ms] cmd_vel 無受信でWatchdog発火
constexpr double DT_SEC = TIMER_INTERVAL_MS * 0.001;
constexpr double ZERO_CMD_EPS = 1e-9;  // 速度指令ゼロ判定の微小しきい値

// ---- 配線の実体（HWオブジェクト） ----
// ピンは素直な順序で束ね、正逆の吸収はDiffDriveのreverse指定に集約する
Encoder enc_l(PIN_ENC_A_L, PIN_ENC_B_L);
Encoder enc_r(PIN_ENC_A_R, PIN_ENC_B_R);
// 左正方向・右反転（回転方向に対するカウント符号を反転）
DiffDrive diff(enc_l, enc_r, false, true);

RcReceiver rc_receiver;
VelocityCalculator vel_calc(COUNTS_PER_REV, SharedParams{}.wheel_radius, DT_SEC);
PIDController pid_right(SharedParams{}.rkp, SharedParams{}.rki, SharedParams{}.rkd, DT_SEC);
PIDController pid_left(SharedParams{}.lkp, SharedParams{}.lki, SharedParams{}.lkd, DT_SEC);
RobotController robot_ctrl(rc_receiver, SharedParams{}.wheel_base, MAX_LINEAR_SPEED, WATCHDOG_TIMEOUT_MS);
MotorDriver motor_l(PIN_PWM_L, PIN_DIR_L);
MotorDriver motor_r(PIN_PWM_R, PIN_DIR_R);
// 右反転は現行MotorControllerのDIR論理と一致
DiffMotors motors(motor_l, motor_r, false, true);
OdometryCalculator odom;

// ---- ループ可変状態（HWオブジェクトと分離して集約） ----
// カウントはint64累積で保持し、int32境界のラップを吸収する（約3.8日問題の対策）
struct LoopState {
    int32_t prev_raw_l = 0;  // 前回生カウント（ラップ検出用）
    int32_t prev_raw_r = 0;
    int64_t cum_l = 0;  // ラップ吸収済み累積カウント
    int64_t cum_r = 0;
    int64_t cum_prev_l = 0;  // 速度計算用前回値（calculateが更新）
    int64_t cum_prev_r = 0;
    double vel_l = 0.0;  // 現在速度 [m/s]
    double vel_r = 0.0;
    SharedParams applied;  // 最後に適用したパラメータ（初期=既定値のため初回適用はskip）
};
LoopState loop_state;

// ---- ステージ間受渡し用の値型 ----
struct MotionSample {
    int32_t count_l = 0;
    int32_t count_r = 0;
};
struct WheelCmd {
    double left = 0.0;  // [m/s]
    double right = 0.0;
};

// ============================================================
// ステージ1：指令受付（mailboxから最新の指令だけ使う）
// ============================================================
void fetchRosCommands() {
    RosVelocityCmd cmd;
    if (g_sys.popRosCmd(cmd)) {
        robot_ctrl.updateRos2Command(cmd.linear_x, cmd.angular_z);
    }
}

// ============================================================
// ステージ2：パラメータ適用（変更時のみ書込む。同一値の再代入を避ける）
// ============================================================
void applyParamsIfChanged(const SharedParams &p) {
    if (std::memcmp(&p, &loop_state.applied, sizeof(p)) == 0) {
        return;
    }
    vel_calc.setWheelRadius(p.wheel_radius);
    robot_ctrl.setWheelBase(p.wheel_base);
    pid_right.setGains(p.rkp, p.rki, p.rkd);
    pid_left.setGains(p.lkp, p.lki, p.lkd);
    loop_state.applied = p;
}

// ============================================================
// ステージ3：状態推定（計数スナップショット→オドメトリ→車輪速度）
// ============================================================
MotionSample estimateMotion(const SharedParams &p) {
    // 反転補正済みスナップショットをDiffDriveから取得する
    MotionSample s;
    diff.snapshot(s.count_l, s.count_r);

    // int32ラップを吸収してint64累積へ
    const int64_t dl = VelocityCalculator::wrapDelta(s.count_l, loop_state.prev_raw_l);
    const int64_t dr = VelocityCalculator::wrapDelta(s.count_r, loop_state.prev_raw_r);
    loop_state.prev_raw_l = s.count_l;
    loop_state.prev_raw_r = s.count_r;
    loop_state.cum_l += dl;
    loop_state.cum_r += dr;

    const double k = (2.0 * PI * p.wheel_radius) / COUNTS_PER_REV;
    odom.setWheelBase(p.wheel_base);
    odom.update(static_cast<double>(dl) * k, static_cast<double>(dr) * k, DT_SEC);

    vel_calc.calculateBothWheels(loop_state.cum_l, loop_state.cum_r,
                                 loop_state.cum_prev_l, loop_state.cum_prev_r,
                                 loop_state.vel_l, loop_state.vel_r);
    return s;
}

// ============================================================
// ステージ4：制御・出力（MANUALは開ループ直結、ROS2はPID閉ループ）
// 直結ゲイン: フルスティック (±MAX_LINEAR_SPEED) → ±DUTY_MAX
// ============================================================
double velCmdToDuty(double vel_cmd) {
    const double duty =
        (vel_cmd / static_cast<double>(MAX_LINEAR_SPEED)) * static_cast<double>(MotorDriver::DUTY_MAX);
    if (duty > static_cast<double>(MotorDriver::DUTY_MAX)) {
        return static_cast<double>(MotorDriver::DUTY_MAX);
    }
    if (duty < -static_cast<double>(MotorDriver::DUTY_MAX)) {
        return -static_cast<double>(MotorDriver::DUTY_MAX);
    }
    return duty;
}

WheelCmd computeAndDrive() {
    const WheelCmd cmd{robot_ctrl.getLeftVelCmd(), robot_ctrl.getRightVelCmd()};
    const bool manual = (robot_ctrl.getControlMode() == RobotController::MODE_MANUAL);

    double l_pwm = 0.0;
    double r_pwm = 0.0;
    if (manual) {
        // 開ループ直結。PID状態は使わないため積分器を保全する
        l_pwm = velCmdToDuty(cmd.left);
        r_pwm = velCmdToDuty(cmd.right);
        pid_left.reset();
        pid_right.reset();
    } else {
        r_pwm = pid_right.compute(cmd.right, loop_state.vel_r, DT_SEC);
        l_pwm = pid_left.compute(cmd.left, loop_state.vel_l, DT_SEC);
    }

    // 速度指令ゼロ時はPWMと積分項をリセット
    if (std::fabs(cmd.right) < ZERO_CMD_EPS) {
        r_pwm = 0.0;
        pid_right.reset();
    }
    if (std::fabs(cmd.left) < ZERO_CMD_EPS) {
        l_pwm = 0.0;
        pid_left.reset();
    }

    motors.setBoth(l_pwm, r_pwm);
    return cmd;
}

// ============================================================
// ステージ5：テレメトリ書込（ros taskが発行する）
// ============================================================
void publishTelemetry(const MotionSample &s, const WheelCmd &cmd) {
    SharedMotion m;
    m.count_l = s.count_l;
    m.count_r = s.count_r;
    m.vel_l = static_cast<float>(loop_state.vel_l);
    m.vel_r = static_cast<float>(loop_state.vel_r);
    m.vel_cmd_l = static_cast<float>(cmd.left);
    m.vel_cmd_r = static_cast<float>(cmd.right);
    m.rc_pulse[0] = rc_receiver.getPulseWidth(CH_LEFT);
    m.rc_pulse[1] = rc_receiver.getPulseWidth(CH_MODE_SW);
    m.rc_pulse[2] = rc_receiver.getPulseWidth(CH_RIGHT);
    m.ctrl_mode = (robot_ctrl.getControlMode() == RobotController::MODE_ROS2) ? 1 : 0;
    m.odom_x = odom.x;
    m.odom_y = odom.y;
    m.odom_theta = odom.theta;
    g_sys.setMotion(m);
}

void control_loop() {
    fetchRosCommands();

    const SharedParams p = g_sys.getParams();
    applyParamsIfChanged(p);

    // RC入力と制御モード更新（調停はRobotControllerに委譲）
    robot_ctrl.update(CH_LEFT, CH_MODE_SW, CH_RIGHT, RC_SIGNAL_TIMEOUT_MS);

    const MotionSample s = estimateMotion(p);
    const WheelCmd cmd = computeAndDrive();
    publishTelemetry(s, cmd);
}

}  // namespace

void controlTask(void *arg) {
    (void)arg;

    // エンコーダー初期化 (PCNTハード計数開始。DiffDrive経由で2軸まとめて)
    diff.begin();

    // モーター制御初期化
    motors.begin(20000, 8);  // 20kHz, 8bit分解能

    // RC レシーバー初期化
    const uint8_t rc_pins[RC_NUM_CHANNELS] = {RC_LEFT_PIN, RC_MODE_SW_PIN, RC_RIGHT_PIN};
    rc_receiver.begin(rc_pins, RC_NUM_CHANNELS);

    // 非常停止は回路実装前のため未配線（TODO.mdで管理）

    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        control_loop();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TIMER_INTERVAL_MS));
    }
}
