#pragma once

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
#include <micro_ros_platformio.h>
#include <Arduino.h>


//エンコーダ用
#define PIN_ENC_A_L 4
#define PIN_ENC_B_L 5
#define PIN_ENC_A_R 13
#define PIN_ENC_B_R 14

//足回り用
#define PIN_DIR_R   25
#define PIN_PWM_R   26
#define PIN_DIR_L   32
#define PIN_PWM_L   33

//ROS用設定
#define ROS_DOMAIN_ID 90
#define WATCHDOG_TIMEOUT 1000

//足回り速度制御用
extern double RKP;
extern double RKI;
extern double RKD;
extern double LKP;
extern double LKI;
extern double LKD;

//車体パラメータ
#define COUNTS_PER_REV    4096.0
extern double WHEEL_RADIUS;  //ホイール径
extern double WHEEL_BASE;  //車輪間幅


//topic通信で使用するメッセージ宣言
extern std_msgs__msg__Int32MultiArray enc_msg;         //エンコーダー情報
extern std_msgs__msg__Float64MultiArray vlt_msg;       //電圧情報
extern std_msgs__msg__Float64MultiArray curr_vel_msg;  //速度情報
extern geometry_msgs__msg__Twist cmd_vel_msg;          //速度指令値
extern mirs_msgs__msg__BasicParam param_msg;           //パラメーターメッセージ
 
 //service通信で使用するメッセージ宣言
extern mirs_msgs__srv__ParameterUpdate_Response update_res;
extern mirs_msgs__srv__ParameterUpdate_Request update_req;
extern mirs_msgs__srv__SimpleCommand_Response reset_res;
extern mirs_msgs__srv__SimpleCommand_Request reset_req;
 
 //publisher,subscriber,serviceの宣言
extern rcl_publisher_t enc_pub;
extern rcl_publisher_t vlt_pub;
extern rcl_publisher_t curr_vel_pub;
extern rcl_subscription_t cmd_vel_sub;
extern rcl_subscription_t param_sub;
extern rcl_service_t update_srv;
extern rcl_service_t reset_srv;
 
 //ノードに関わる宣言
extern rclc_executor_t executor;
extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern rcl_node_t node;
extern rcl_timer_t timer;

/* 処理で使用するグローバル変数 */

//エンコーダーカウント
extern int32_t count_l;
extern int32_t count_r;

extern int32_t prev_count_l;
extern int32_t prev_count_r;

//速度制御用の変数
extern double r_vel_cmd;
extern double l_vel_cmd;
extern double r_vel;
extern double l_vel;

extern float linear_x;   //  直進速度
extern float angular_z;  //  回転速度

extern float r_err_sum;
extern float l_err_sum;

extern float prev_r_err;
extern float prev_l_err;

//WatchDog用
extern uint32_t lastCalledAt;

extern unsigned long last_loop_time;

extern int pwmFrequency;
extern int pwmResolution;

void ros_setup();
void encoder_open();
void vel_ctrl_set();
void timer_callback(rcl_timer_t*, int64_t);
void rosid_setup_jazzy();
void PID_control();
void encoder_reset();
void cmd_vel_Callback(const void *);
void reset_service_callback(const void *, void *);
void update_service_callback(const void *, void *);
void param_service_callback(const void *, void *);
void param_Callback(const void *);
