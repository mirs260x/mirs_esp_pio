#include "control_task.hpp"
#include <Arduino.h>
#include <cmath>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "hardware_config.hpp"
#include "Encoder.hpp"
#include "DiffDrive.hpp"
#include "PoseEstimator.hpp"
#include "RcReceiver.hpp"
#include "PIDController.hpp"
#include "RobotController.hpp"
#include "MotorDriver.hpp"
#include "DiffMotors.hpp"
#include "SystemContext.hpp"
#include <atomic>

// rosタスクからのリセット要求フラグ。control周期内で消費する。
static std::atomic<bool> odom_reset_req{false};

void requestOdometryReset() {
    odom_reset_req.store(true);
}

// 配線と実行順序のみ持つ。計算・調停・出力・共有は各層に委譲する。
// パイプライン: 指令受付→パラメータ適用→調停→状態推定→制御・出力→テレメトリ
namespace {

// 設定値の唯一の出所はSharedParams既定値。ここでの二重定義はしない
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 1000;  // [ms] cmd_vel 無受信でWatchdog発火
constexpr double DT_SEC = TIMER_INTERVAL_MS * 0.001;  // [s] 制御周期
constexpr double ZERO_CMD_EPS = 1e-9;  // [m/s] 速度指令ゼロ判定しきい値

// 配線の実体（HWオブジェクト）。正逆の吸収はDiffDrive/DiffMotorsのreverse指定に集約する
Encoder enc_l(PIN_ENC_A_L, PIN_ENC_B_L);
Encoder enc_r(PIN_ENC_A_R, PIN_ENC_B_R);
DiffDrive diff(enc_l, enc_r, false, true);  // 左正方向・右反転

RcReceiver rc_receiver;
PIDController pid_right(SharedParams{}.rkp, SharedParams{}.rki, SharedParams{}.rkd, DT_SEC);
PIDController pid_left(SharedParams{}.lkp, SharedParams{}.lki, SharedParams{}.lkd, DT_SEC);
RobotController robot_ctrl(rc_receiver, SharedParams{}.wheel_base, MAX_LINEAR_SPEED, WATCHDOG_TIMEOUT_MS);
MotorDriver motor_l(PIN_PWM_L, PIN_DIR_L);
MotorDriver motor_r(PIN_PWM_R, PIN_DIR_R);
DiffMotors motors(motor_l, motor_r, false, true);  // 右反転はDIR論理と一致
// 状態推定ファサード。odom/EKF/速度の判断は内包し、タスク側は投入と取出しのみ行う
PoseEstimator estimator(COUNTS_PER_REV, SharedParams{}.wheel_radius,
                        SharedParams{}.wheel_base, ENABLE_EKF == 1);

// 最後に適用したパラメータ（初期=既定値のため初回適用はskip）
SharedParams applied_params;

// ステージ間受渡し用の値型
struct MotionSample {
    int32_t count_l = 0;
    int32_t count_r = 0;
};
struct WheelCmd {
    double left = 0.0;  // [m/s]
    double right = 0.0;
};

// ステージ1：指令受付（mailboxから最新の指令だけ使う）
void fetchRosCommands() {
    RosVelocityCmd cmd;
    if (g_sys.popRosCmd(cmd)) {
        robot_ctrl.updateRos2Command(cmd.linear_x, cmd.angular_z, cmd.stamp_ms);
    }
}

// ステージ2：パラメータ適用（変更時のみ書込む）。
// 適用先の一覧はここに集約する。新規consumer追加時は本関数への登録を忘れないこと。
// 現行：estimator(半径/base)/robot_ctrl(diff逆運動学)/diff(距離)/pid×2のgain
void applyParamsIfChanged(const SharedParams &p) {
    if (p == applied_params) {
        return;
    }
    estimator.applyParams(p.wheel_radius, p.wheel_base);
    robot_ctrl.setWheelBase(p.wheel_base);
    diff.setWheelParams(p.wheel_radius, p.wheel_base);
    pid_right.setGains(p.rkp, p.rki, p.rkd);
    pid_left.setGains(p.lkp, p.lki, p.lkd);
    applied_params = p;
}

// SharedSensor→推定器入力の変換（配線）。FW非依存の値型に詰め替えるだけ。
EstimatorImu toEstimatorImu(const SharedSensor &s) {
    EstimatorImu imu;
    imu.imu_ok = s.imu_ok;
    imu.mag_ok = s.mag_ok;
    imu.ax = s.ax;
    imu.ay = s.ay;
    imu.az = s.az;
    imu.gz = s.gz;
    imu.mx = s.mx;
    imu.my = s.my;
    imu.mz = s.mz;
    return imu;
}

// ステージ3：状態推定。読取→投入→結果取出しのみ。判断はPoseEstimatorに委譲する。
MotionSample estimateMotion(double dt_sec) {
    // 単一読取でスナップショット・差分・移動距離を同時更新する（二重読み防止）
    MotionSample s;
    int64_t dl = 0, dr = 0;
    diff.sample(s.count_l, s.count_r, dl, dr);
    estimator.update(dl, dr, diff.distLeft(), diff.distRight(),
                     toEstimatorImu(g_sys.getSensor()), dt_sec);
    return s;
}

// ステージ4：制御・出力（MANUALは開ループ直結、ROS2はPID閉ループ）。
// 速度→duty換算はRobotController、極性吸収はDiffMotorsに委譲する。
WheelCmd computeAndDrive(double dt_sec) {
    const WheelCmd cmd{robot_ctrl.getLeftVelCmd(), robot_ctrl.getRightVelCmd()};
    const bool manual = (robot_ctrl.getControlMode() == RobotController::MODE_MANUAL);

    double l_pwm = 0.0;
    double r_pwm = 0.0;
    if (manual) {
        // 開ループ直結。PID状態は使わないため積分器を保全する
        l_pwm = robot_ctrl.toDuty(cmd.left);
        r_pwm = robot_ctrl.toDuty(cmd.right);
        pid_left.reset();
        pid_right.reset();
    } else {
        r_pwm = pid_right.compute(cmd.right, estimator.velR(), dt_sec);
        l_pwm = pid_left.compute(cmd.left, estimator.velL(), dt_sec);
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

// ステージ5：テレメトリ書込（ros taskが発行する）。
// 推定結果の取出しと共有構造体への詰替えのみ。判断は持たない。
void publishTelemetry(const MotionSample &s, const WheelCmd &cmd) {
    SharedMotion m;
    m.count_l = s.count_l;
    m.count_r = s.count_r;
    m.vel_l = estimator.velL();
    m.vel_r = estimator.velR();
    m.vel_cmd_l = static_cast<float>(cmd.left);
    m.vel_cmd_r = static_cast<float>(cmd.right);
    m.rc_pulse[0] = rc_receiver.getPulseWidth(CH_LEFT);
    m.rc_pulse[1] = rc_receiver.getPulseWidth(CH_MODE_SW);
    m.rc_pulse[2] = rc_receiver.getPulseWidth(CH_RIGHT);
    m.ctrl_mode = (robot_ctrl.getControlMode() == RobotController::MODE_ROS2) ? 1 : 0;
    m.odom_x = estimator.x();
    m.odom_y = estimator.y();
    m.odom_theta = estimator.theta();
    m.lin_vel = estimator.v();
    m.ang_vel = estimator.w();
    g_sys.setMotion(m);
}

void control_loop(double dt_sec) {
    fetchRosCommands();

    const SharedParams p = g_sys.getParams();
    applyParamsIfChanged(p);

    // RC入力と制御モード更新（調停はRobotControllerに委譲）
    robot_ctrl.update(CH_LEFT, CH_MODE_SW, CH_RIGHT, RC_SIGNAL_TIMEOUT_MS);

    const MotionSample s = estimateMotion(dt_sec);
    // 原点リセット要求があれば積算後にゼロ化する（参照継続のため速度に段差なし）。
    // SLAM開始前の停止状態で使うこと。走行中の呼出しは軌跡を切断する。
    if (odom_reset_req.exchange(false)) {
        estimator.reset();
    }
    const WheelCmd cmd = computeAndDrive(dt_sec);
    publishTelemetry(s, cmd);
}

}  // namespace

void controlTask(void *arg) {
    (void)arg;

    // エンコーダー初期化 (GPIO割込み計数開始。DiffDrive経由で2軸まとめて)
    diff.begin();

    // モーター制御初期化
    motors.begin(20000, 8);  // 20kHz, 8bit分解能

    // RC レシーバー初期化
    const uint8_t rc_pins[RC_NUM_CHANNELS] = {RC_LEFT_PIN, RC_MODE_SW_PIN, RC_RIGHT_PIN};
    rc_receiver.begin(rc_pins, RC_NUM_CHANNELS);

    // 非常停止は回路実装前のため未配線（TODO.mdで管理）

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t prev_tick = last_wake;
    for (;;) {
        // 実測周期で速度・オドメ・PIDを駆動する（ジッタを無視しない）
        const TickType_t now_tick = xTaskGetTickCount();
        double dt_sec = static_cast<double>(now_tick - prev_tick) /
                        static_cast<double>(configTICK_RATE_HZ);
        prev_tick = now_tick;
        if (dt_sec <= 0.0) {
            dt_sec = DT_SEC;
        }
        control_loop(dt_sec);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TIMER_INTERVAL_MS));
    }
}
