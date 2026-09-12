#pragma once

#include <Arduino.h>

// Hardware-only configuration. This header must not depend on micro-ROS.
// エンコーダピン。左右の正逆の違い（ミラー実装）はソフトウェア側で吸収する。
// DifferentialDrive生成時のreverse指定（左=true）で対応し、
// ハード変更時はその指定だけ変える。define値自体は変えないこと。
#define PIN_ENC_A_R 4
#define PIN_ENC_B_R 5
#define PIN_ENC_A_L 13
#define PIN_ENC_B_L 14
#define PIN_DIR_R 32
#define PIN_PWM_R 33
#define PIN_DIR_L 25
#define PIN_PWM_L 26
// エンコーダー 1回転あたりのカウント数 (4逓倍 × 1024パルス/rev = 4096)
// CugoParams::ENCODER_RESOLUTION (2048) は未使用のため削除済み
#define COUNTS_PER_REV 4096.0
#define ESTOP_PIN 19 // GPIO34は内部プルアップ非対応のため変更

#define PIN_BATT_1 36
#define PIN_BATT_2 39
#define VOLTAGE_DIVIDER_RATIO 5.545f
#define ADC_REF_VOLTAGE 3.3f
#define ADC_RESOLUTION 4095.0f

// IMU (BMX055) I2C Pins
#define PIN_IMU_SDA 18
#define PIN_IMU_SCL 27

// 制御・ROSタイマー共通周期 [ms]（control task周期とros timer周期で共有）
#define TIMER_INTERVAL_MS 15

#define RC_LEFT_PIN    21
#define RC_MODE_SW_PIN 22
#define RC_RIGHT_PIN   23
#define RC_NUM_CHANNELS 3
#define CH_LEFT     0
#define CH_MODE_SW  1
#define CH_RIGHT    2
#define RC_PULSE_MIN 1000
#define RC_PULSE_MID 1500
#define RC_PULSE_MAX 2000
#define RC_PULSE_DEADZONE 40
#define RC_SIGNAL_TIMEOUT_MS 100
#define MAX_LINEAR_SPEED 0.8f
#define MAX_ANGULAR_SPEED 1.5f
