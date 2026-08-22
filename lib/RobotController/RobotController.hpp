/**
 * @file RobotController.hpp
 * @brief ロボット上位制御クラス
 * 
 * RC受信機とROS2からの速度指令を統合管理します。
 * - 制御モード切替（手動/ROS2）
 * - ウォッチドッグ機能
 * - 速度指令の生成と管理
 */

#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <Arduino.h>
#include "RcReceiver.hpp"

class RobotController {
public:
    /// 制御モード
    enum ControlMode : uint8_t {
        MODE_MANUAL,  ///< 手動制御モード（RC入力）
        MODE_ROS2     ///< ROS2制御モード
    };

    /**
     * @brief コンストラクタ
     * @param rc_receiver RC受信機オブジェクトへの参照
     * @param wheel_base 車輪間距離 [m]
     * @param max_linear_speed 最大直進速度 [m/s]
     * @param watchdog_timeout ウォッチドッグタイムアウト [ms]
     */
    RobotController(
        RcReceiver &rc_receiver,
        double wheel_base,
        float max_linear_speed,
        uint32_t watchdog_timeout
    );

    /**
     * @brief 車輪間距離を設定
     * @param wheel_base 車輪間距離 [m]
     */
    void setWheelBase(double wheel_base);

    /**
     * @brief 制御モードを取得
     * @return 現在の制御モード
     */
    ControlMode getControlMode() const { return control_mode_; }

    /**
     * @brief 制御モードを設定
     * @param mode 制御モード
     */
    void setControlMode(ControlMode mode) { control_mode_ = mode; }

    /**
     * @brief ROS2からの速度指令を更新
     * @param linear_x 直進速度 [m/s]
     * @param angular_z 角速度 [rad/s]
     * 
     * この関数を呼び出すとウォッチドッグタイマーがリセットされます。
     */
    void updateRos2Command(float linear_x, float angular_z);

    /**
     * @brief RC入力とモード切替を処理して速度指令を更新
     * @param ch_left RC左チャンネル番号
     * @param ch_mode_sw RCモード切替スイッチチャンネル番号
     * @param ch_right RC右チャンネル番号
     * @param rc_signal_timeout RC信号タイムアウト [ms]
     * 
     * 手動モード時はRC入力から速度指令を生成します。
     * ROS2モード時はウォッチドッグをチェックします。
     * モード切替スイッチの立ち上がりエッジでモードをトグルします。
     */
    void update(uint8_t ch_left, uint8_t ch_mode_sw, uint8_t ch_right, uint32_t rc_signal_timeout);

    /**
     * @brief 左輪速度指令を取得
     * @return 左輪速度指令 [m/s]
     */
    double getLeftVelCmd() const { return l_vel_cmd_; }

    /**
     * @brief 右輪速度指令を取得
     * @return 右輪速度指令 [m/s]
     */
    double getRightVelCmd() const { return r_vel_cmd_; }

    /**
     * @brief ROS2からの直進速度指令を取得
     * @return 直進速度指令 [m/s]
     */
    float getLinearX() const { return linear_x_; }

    /**
     * @brief ROS2からの角速度指令を取得
     * @return 角速度指令 [rad/s]
     */
    float getAngularZ() const { return angular_z_; }

private:
    RcReceiver &rc_receiver_;        ///< RC受信機への参照
    double wheel_base_;              ///< 車輪間距離 [m]
    float max_linear_speed_;         ///< 最大直進速度 [m/s]
    uint32_t watchdog_timeout_;      ///< ウォッチドッグタイムアウト [ms]
    
    ControlMode control_mode_;       ///< 現在の制御モード
    float linear_x_;                 ///< ROS2直進速度指令 [m/s]
    float angular_z_;                ///< ROS2角速度指令 [rad/s]
    double l_vel_cmd_;               ///< 左輪速度指令 [m/s]
    double r_vel_cmd_;               ///< 右輪速度指令 [m/s]
    uint32_t last_ros2_cmd_time_;    ///< 最後にROS2指令を受信した時刻 [ms]
    bool prev_sw_high_;              ///< モードスイッチの前回状態（エッジ検出用）

    /**
     * @brief RC入力から速度指令を生成（手動モード）
     * @param ch_left RC左チャンネル番号
     * @param ch_right RC右チャンネル番号
     */
    void updateManualCommand(uint8_t ch_left, uint8_t ch_right);

    /**
     * @brief ROS2速度指令から左右輪速度を計算（ROS2モード）
     */
    void updateRos2WheelCommands();

    /**
     * @brief ウォッチドッグチェック（ROS2モード）
     * 
     * タイムアウト時は速度指令をゼロにします。
     */
    void checkWatchdog();
};

#endif // ROBOT_CONTROLLER_HPP
