#pragma once

#include <Arduino.h>

// Hardware-only configuration. This header must not depend on micro-ROS.
// エンコーダピン。左右の正逆の違いはソフトウェア側で吸収する。
// DiffDrive生成時のreverse指定で対応し、
// ハード変更時はその指定だけ変える。define値自体は変えないこと。
#define PIN_ENC_A_R 4
#define PIN_ENC_B_R 5
#define PIN_ENC_A_L 13
#define PIN_ENC_B_L 14
#define PIN_DIR_R 32
#define PIN_PWM_R 33
#define PIN_DIR_L 25
#define PIN_PWM_L 26
// エンコーダー 1回転あたりのカウント数 (X4換算 × 1024パルス/rev = 4096。車輪1回転あたり実測)
#define COUNTS_PER_REV 4096.0

#define PIN_BATT_1 36
#define PIN_BATT_2 39
#define VOLTAGE_DIVIDER_RATIO 5.545f
#define ADC_REF_VOLTAGE 3.3f
#define ADC_RESOLUTION 4095.0f

// IMU (BMX055) I2C Pins
#define PIN_IMU_SDA 18
#define PIN_IMU_SCL 27

// IMUの有無は実行時に自動判定する（不在時はプラグイン無効化＋無発行）。
// プリプロセッサ分離はしない方針のため ENABLE_IMU は廃止した。
// 除去時は ImuPlugin の登録1行＋ extra_packages/imu を外すだけ。

// EKF有効化フラグ。0=無効（従来どおりエンコーダのみのオドメトリ）。
// 1にすると control タスク内で Mirs2605Ekf が IMU＋エンコーダ＋地磁気を融合し、
// /odom の pose/velocity をフィルタ結果で上書きする。ROS側 robot_localization は残すこと
//（ESP32は前段フィルタ、ROSは後段融合の二段構成）。
#define ENABLE_EKF 0

// ファームウェア版数。起動時シリアルに出力する（書込み確認用）。修正時は更新すること。
#define FW_VERSION "0.5.0"

// 制御・ROSタイマー共通周期 [ms]（control task周期とros timer周期で共有）
#define TIMER_INTERVAL_MS 15

#define RC_LEFT_PIN    21
#define RC_MODE_SW_PIN 22
#define RC_RIGHT_PIN   23
#define RC_NUM_CHANNELS 3
#define CH_LEFT     0
#define CH_MODE_SW  1
#define CH_RIGHT    2
#define RC_PULSE_MIN 890
#define RC_PULSE_MID 1496
#define RC_PULSE_MAX 2100
#define RC_PULSE_DEADZONE 40
#define RC_SIGNAL_TIMEOUT_MS 100
#define MAX_LINEAR_SPEED 0.8f
#define MAX_ANGULAR_SPEED 1.5f
