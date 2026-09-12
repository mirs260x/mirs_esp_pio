#include "control_task.hpp"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "hardware_config.hpp"
#include "RcReceiver.hpp"
#include "VelocityCalculator.hpp"
#include "PIDController.hpp"
#include "RobotController.hpp"
#include "MotorController.hpp"
#include "SafetyEstop.hpp"
#include "SystemContext.hpp"

// ============================================================
// 設定（旧main.cppから移管。振る舞い同一）
// ============================================================
#define WATCHDOG_TIMEOUT 1000  // [ms] cmd_vel 無受信でWatchdog発火

// PIDゲイン初期値（/params受信で上書きされる。SharedParams既定値と一致）
static double RKP = 80.0, RKI = 30.0, RKD = 8.0;
static double LKP = 80.0, LKI = 30.0, LKD = 8.0;

// ============================================================
// 制御側状態（旧main.cppグローバルを移管）
// ============================================================
static volatile int32_t count_l = 0, count_r = 0;  // エンコーダーカウント
static double r_vel = 0, l_vel = 0;                // 現在速度 [m/s]
static int32_t prev_count_l = 0, prev_count_r = 0;

static RcReceiver rc_receiver;
static SafetyEstop safety_estop;
static VelocityCalculator vel_calc(COUNTS_PER_REV, 0.04, TIMER_INTERVAL_MS * 0.001);
static PIDController pid_right(RKP, RKI, RKD);
static PIDController pid_left(LKP, LKI, LKD);
static RobotController robot_ctrl(rc_receiver, 0.38, MAX_LINEAR_SPEED, WATCHDOG_TIMEOUT);
static MotorController motor_ctrl(PIN_DIR_L, PIN_PWM_L, PIN_DIR_R, PIN_PWM_R);

static portMUX_TYPE controlMux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// エンコーダー割り込みハンドラ（旧main.cppと同一式）
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
// 制御ループ処理（旧main.cpp control_loop()と同一）
// ============================================================
static void control_loop() {
    // ROS指令キューを draining し、最新の指令だけ使う
    RosVelocityCmd ros_cmd;
    bool fresh = false;
    while (g_sys.popRosCmd(ros_cmd)) {
        fresh = true;
    }
    if (fresh) {
        robot_ctrl.updateRos2Command(ros_cmd.linear_x, ros_cmd.angular_z);
    }

    // /params反映（毎周期そのまま書込む。同一値の再代入は無害）
    const SharedParams p = g_sys.getParams();
    vel_calc.setWheelRadius(p.wheel_radius);
    robot_ctrl.setWheelBase(p.wheel_base);
    pid_right.setGains(p.rkp, p.rki, p.rkd);
    pid_left.setGains(p.lkp, p.lki, p.lkd);

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

    // テレメトリ書込（ros taskが発行する）
    SharedMotion m;
    m.count_l = snap_l;
    m.count_r = snap_r;
    m.vel_l = static_cast<float>(l_vel);
    m.vel_r = static_cast<float>(r_vel);
    m.vel_cmd_l = static_cast<float>(l_vel_cmd);
    m.vel_cmd_r = static_cast<float>(r_vel_cmd);
    m.rc_pulse[0] = rc_receiver.getPulseWidth(CH_LEFT);
    m.rc_pulse[1] = rc_receiver.getPulseWidth(CH_MODE_SW);
    m.rc_pulse[2] = rc_receiver.getPulseWidth(CH_RIGHT);
    m.ctrl_mode = (robot_ctrl.getControlMode() == RobotController::MODE_ROS2) ? 1 : 0;
    g_sys.setMotion(m);
}

void controlTask(void *arg) {
    (void)arg;

    // エンコーダー初期化
    for (uint8_t pin : {PIN_ENC_A_L, PIN_ENC_B_L, PIN_ENC_A_R, PIN_ENC_B_R}) {
        pinMode(pin, INPUT_PULLUP);
    }
    attachInterrupt(PIN_ENC_A_L, enc_change_l, CHANGE);
    attachInterrupt(PIN_ENC_A_R, enc_change_r, CHANGE);

    // モーター制御初期化
    motor_ctrl.begin(20000, 8);  // 20kHz, 8bit分解能

    // RC レシーバー初期化
    const uint8_t rc_pins[RC_NUM_CHANNELS] = {RC_LEFT_PIN, RC_MODE_SW_PIN, RC_RIGHT_PIN};
    rc_receiver.begin(rc_pins, RC_NUM_CHANNELS);

    // 非常停止初期化 (回路実装前のため一旦無効化)
    // safety_estop.begin(ESTOP_PIN);

    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        control_loop();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TIMER_INTERVAL_MS));
    }
}
