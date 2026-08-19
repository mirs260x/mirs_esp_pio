#include "config.hpp"
#include <micro_ros_platformio.h>
#include <Arduino.h>
#include <micro_ros_platformio.h>
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
 
void encoder_reset() {
  count_l = 0;
  count_r = 0;
  prev_count_l = 0;
  prev_count_r = 0;
  r_vel = 0.0;
  l_vel = 0.0;
  r_vel_cmd = 0.0;
  l_vel_cmd = 0.0;
  r_err_sum = 0.0f;
  l_err_sum = 0.0f;
  prev_r_err = 0.0f;
  prev_l_err = 0.0f;
  last_velocity_update_ms = millis();
}

static void enc_change_l() {
  int32_t a_curr = digitalRead(PIN_ENC_A_L);
  int32_t b_curr = digitalRead(PIN_ENC_B_L);

  if(a_curr == b_curr){
    count_l--;
  }else{
    count_l++;
  }
}

static void enc_change_r() {
  int32_t a_curr = digitalRead(PIN_ENC_A_R);
  int32_t b_curr = digitalRead(PIN_ENC_B_R);

  if(a_curr == b_curr){
    count_r++;
  }else{
    count_r--;
  }
}

void encoder_open() {
  pinMode(PIN_ENC_A_L, INPUT);
  pinMode(PIN_ENC_B_L, INPUT);
  pinMode(PIN_ENC_A_R, INPUT);
  pinMode(PIN_ENC_B_R, INPUT);
  digitalWrite(PIN_ENC_A_L, HIGH);
  digitalWrite(PIN_ENC_B_L, HIGH);
  digitalWrite(PIN_ENC_A_R, HIGH);
  digitalWrite(PIN_ENC_B_R, HIGH);
  attachInterrupt(PIN_ENC_A_L, enc_change_l, CHANGE);
  attachInterrupt(PIN_ENC_A_R, enc_change_r, CHANGE);

  enc_msg.data.capacity = 2;
  enc_msg.data.size = 2;
  enc_msg.data.data = (int32_t *)malloc(enc_msg.data.capacity * sizeof(int32_t)); // 配列のメモリを確保
  enc_msg.data.data[0] = 0;
  enc_msg.data.data[1] = 0;
}
