/**
 * @file RobotController.cpp
 * @brief RobotControllerクラスの実装
 */

#include "RobotController.hpp"

RobotController::RobotController(
    RcReceiver &rc_receiver,
    double wheel_base,
    float max_linear_speed,
    uint32_t watchdog_timeout
)
    : rc_receiver_(rc_receiver)
    , wheel_base_(wheel_base)
    , max_linear_speed_(max_linear_speed)
    , watchdog_timeout_(watchdog_timeout)
    , control_mode_(MODE_MANUAL)
    , linear_x_(0.0f)
    , angular_z_(0.0f)
    , l_vel_cmd_(0.0)
    , r_vel_cmd_(0.0)
    , last_ros2_cmd_time_(0)
    , prev_sw_high_(false)
{
}

void RobotController::setWheelBase(double wheel_base) {
    wheel_base_ = wheel_base;
}

void RobotController::updateRos2Command(float linear_x, float angular_z, uint32_t stamp_ms) {
    linear_x_ = linear_x;
    angular_z_ = angular_z;
    // mailbox滞留分だけwatchdog判定が遅れないよう、受信時刻ではなく指令生成時刻で更新する
    last_ros2_cmd_time_ = (stamp_ms != 0) ? stamp_ms : millis();
}

void RobotController::update(uint8_t ch_left, uint8_t ch_mode_sw, uint8_t ch_right, uint32_t rc_signal_timeout) {
    // モードSWの立ち上がりエッジ検出（0 -> 1でモードトグル）。
    // ROS2からのトグル先は必ずMANUALのため、ROS2走行中の押下は必ず制御権を奪取する。
    // SW信号が無効な間はトグル判定を凍結し、現モードを維持する
    // SW仕様: OFF=1496us、正規化0.0 / ON=1995us、正規化約0.82。閾値0.2 (約1617us) で判定
    if (rc_receiver_.isSignalValid(ch_mode_sw, rc_signal_timeout)) {
        bool current_sw_high =
            (RcReceiver::pulseToNormalized(rc_receiver_.getPulseWidth(ch_mode_sw)) > 0.2f);
        if (current_sw_high && !prev_sw_high_) {
            // 立ち上がりエッジ検出：モードを反転
            control_mode_ = (control_mode_ == MODE_MANUAL) ? MODE_ROS2 : MODE_MANUAL;
        }
        prev_sw_high_ = current_sw_high;
    }

    // 制御モードに応じて速度指令を更新
    if (control_mode_ == MODE_ROS2) {
        // ROS2モードはRC不要。停止判定はwatchdogに一本化する
        // watchdog発火時は0維持し、古いlinear_x_/angular_z_で上書きしない
        if (!checkWatchdog()) {
            updateRos2WheelCommands();
        }
    } else {
        // MANUALモードはRC必須。信号喪失時は停止する
        const bool rc_valid =
            rc_receiver_.isSignalValid(ch_left, rc_signal_timeout) &&
            rc_receiver_.isSignalValid(ch_mode_sw, rc_signal_timeout) &&
            rc_receiver_.isSignalValid(ch_right, rc_signal_timeout);
        if (!rc_valid) {
            r_vel_cmd_ = 0.0;
            l_vel_cmd_ = 0.0;
            return;
        }
        updateManualCommand(ch_left, ch_right);
    }
}

void RobotController::updateManualCommand(uint8_t ch_left, uint8_t ch_right) {
    // RC入力を正規化値（-1.0 ~ +1.0）に変換
    float l_norm = RcReceiver::pulseToNormalized(rc_receiver_.getPulseWidth(ch_left));
    float r_norm = RcReceiver::pulseToNormalized(rc_receiver_.getPulseWidth(ch_right));

    // リニア割当: 左右スティックで左右輪を個別操作
    l_vel_cmd_ = l_norm * max_linear_speed_;
    r_vel_cmd_ = r_norm * max_linear_speed_;
}

void RobotController::updateRos2WheelCommands() {
    // 差動駆動ロボットの逆運動学
    // v_r = v + (w * L) / 2
    // v_l = v - (w * L) / 2
    // ここで v = linear_x, w = angular_z, L = wheel_base
    r_vel_cmd_ = linear_x_ + (wheel_base_ / 2.0) * angular_z_;
    l_vel_cmd_ = linear_x_ - (wheel_base_ / 2.0) * angular_z_;
}

bool RobotController::checkWatchdog() {
    // ROS2モード時のウォッチドッグチェック
    if ((millis() - last_ros2_cmd_time_) > watchdog_timeout_) {
        // タイムアウト：速度指令をゼロに
        r_vel_cmd_ = 0.0;
        l_vel_cmd_ = 0.0;
        return true;
    }
    return false;
}
