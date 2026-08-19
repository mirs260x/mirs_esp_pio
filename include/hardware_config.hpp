#pragma once

#include <Arduino.h>

// Hardware-only configuration. This header must not depend on micro-ROS.
#define PIN_ENC_A_L 4
#define PIN_ENC_B_L 5
#define PIN_ENC_A_R 13
#define PIN_ENC_B_R 14
#define PIN_DIR_R 25
#define PIN_PWM_R 26
#define PIN_DIR_L 32
#define PIN_PWM_L 33
#define COUNTS_PER_REV 4096.0
#define ESTOP_PIN 19 // GPIO34は内部プルアップ非対応のため変更

#define PIN_BATT_1 36
#define PIN_BATT_2 39
#define VOLTAGE_DIVIDER_RATIO 5.545f
#define ADC_REF_VOLTAGE 3.3f
#define ADC_RESOLUTION 4095.0f

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

namespace CugoParams {
constexpr float ENCODER_RESOLUTION = 2048.0f;
constexpr float REDUCTION_RATIO = 1.0f;
constexpr float WHEEL_RADIUS_L = 0.03858f;
constexpr float WHEEL_RADIUS_R = 0.03858f;
constexpr float TREAD = 0.380f;
}
