#include <micro_ros_platformio.h>
#include <Arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <mirs_msgs/srv/parameter_update.h>
#include <mirs_msgs/srv/simple_command.h>
#include <mirs_msgs/msg/basic_param.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <mirs_msgs/action/trigger.h>
#include <pthread.h>
#include "config.hpp"
#include "VoltageSensor.hpp"
#include "RcReceiver.hpp"
#include "SafetyEstop.hpp"

static void update_rc_command() {
  const bool valid =
    rc_receiver.isSignalValid(CH_THROTTLE, RC_SIGNAL_TIMEOUT_MS) &&
    rc_receiver.isSignalValid(CH_STEER, RC_SIGNAL_TIMEOUT_MS) &&
    rc_receiver.isSignalValid(CH_MODE_SW, RC_SIGNAL_TIMEOUT_MS);
  if (!valid) {
    r_vel_cmd = 0.0;
    l_vel_cmd = 0.0;
    return;
  }

  const float mode = RcReceiver::pulseToNormalized(
    rc_receiver.getPulseWidth(CH_MODE_SW));
  control_mode = mode > 0.2f ? MODE_ROS2 : MODE_MANUAL;
  if (control_mode == MODE_ROS2) {
    r_vel_cmd = linear_x + WHEEL_BASE / 2 * angular_z;
    l_vel_cmd = linear_x - WHEEL_BASE / 2 * angular_z;
    return;
  }

  const float linear = RcReceiver::pulseToNormalized(
    rc_receiver.getPulseWidth(CH_THROTTLE)) * MAX_LINEAR_SPEED;
  const float angular = RcReceiver::pulseToNormalized(
    rc_receiver.getPulseWidth(CH_STEER)) * MAX_ANGULAR_SPEED;
  r_vel_cmd = linear + WHEEL_BASE * 0.5 * angular;
  l_vel_cmd = linear - WHEEL_BASE * 0.5 * angular;
}
//ノードのタイマーコールバック関数
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  update_rc_command();

  // ROS2モード時だけ、/cmd_velのウォッチドッグを適用する。
  // 手動モードではMR-8の入力自体を安全監視する。
  if (control_mode == MODE_ROS2 &&
      (millis() - lastCalledAt) > WATCHDOG_TIMEOUT) {
    r_vel_cmd = 0;
    l_vel_cmd = 0;
  }

  //PID計算
  PID_control();

  //エンコーダーデータを格納（モードに関わらず常に送信）
  enc_msg.data.data[0] = count_l;
  enc_msg.data.data[1] = count_r;

  curr_vel_msg.data.data[0] = l_vel;
  curr_vel_msg.data.data[1] = r_vel;
  static uint8_t telemetry_divider = 0;
  const bool publish_telemetry = (++telemetry_divider >= 4);
  if (publish_telemetry) telemetry_divider = 0;
  float voltage_1 = 0.0f;
  float voltage_2 = 0.0f;
  if (ENABLE_VOLTAGE_CUTOFF || publish_telemetry) {
    voltage_1 = voltage_sensor_1.readVoltage();
    voltage_2 = voltage_sensor_2.readVoltage();
    if (ENABLE_VOLTAGE_CUTOFF &&
        (voltage_1 < VOLTAGE_CUTOFF_THRESHOLD ||
         voltage_2 < VOLTAGE_CUTOFF_THRESHOLD)) {
      SafetyEstop::trigger();
    }
  }
  if (publish_telemetry) {
    vlt_msg.data.data[0] = voltage_1;
    vlt_msg.data.data[1] = voltage_2;
    (void)rcl_publish(&enc_pub, &enc_msg, NULL);
    (void)rcl_publish(&vlt_pub, &vlt_msg, NULL);
    (void)rcl_publish(&curr_vel_pub, &curr_vel_msg, NULL);
  }
}

// cmd_velメッセージのコールバック関数
void cmd_vel_Callback(const void * msgin) {
  const geometry_msgs__msg__Twist * vel_msg = (const geometry_msgs__msg__Twist *)msgin;

  // linear.x と angular.z のデータを取得
  linear_x = vel_msg->linear.x;
  angular_z = vel_msg->angular.z;

  //  目標速度計算
  r_vel_cmd = linear_x + WHEEL_BASE / 2 * angular_z;
  l_vel_cmd = linear_x - WHEEL_BASE / 2 * angular_z;

  // WatchDog用 最後に呼び出された時間を格納
  lastCalledAt = millis();
}

//TODO: 消してサービスに移行
//パラメーター更新のコールバック関数
void param_Callback(const void * msgin){
  const mirs_msgs__msg__BasicParam * param_msg = (const mirs_msgs__msg__BasicParam *)msgin;

  // linear.x と angular.z のデータを取得
  WHEEL_RADIUS = param_msg->wheel_radius;
  WHEEL_BASE = param_msg->wheel_base;
  RKP = param_msg->rkp;
  RKI = param_msg->rki;
  RKD = param_msg->rkd;
  LKP = param_msg->lkp;
  LKI = param_msg->lki;
  LKD = param_msg->lkd;
}
//
// //ノードのタイマーコールバック関数
// void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
// {  
//   RCLC_UNUSED(last_call_time);
//   if (timer != NULL) {
//     //PID計算
//     PID_control();
//     //エンコーダーデータを格納
//     enc_msg.data.data[0] = count_l;
//     enc_msg.data.data[1] = count_r;
//     // watchdog
//     if( (millis() - lastCalledAt) > WATCHDOG_TIMEOUT){
//       r_vel_cmd = 0;
//       l_vel_cmd = 0;
//     }
//
//     curr_vel_msg.data.data[0] = l_vel;
//     curr_vel_msg.data.data[1] = r_vel;
//     rcl_publish(&enc_pub, &enc_msg, NULL);
//     rcl_publish(&curr_vel_pub, &curr_vel_msg, NULL);
//   }
// }
//
