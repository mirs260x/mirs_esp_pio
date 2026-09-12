/**
 * @file MotorController.hpp
 * @brief モーター制御クラス
 * 
 * 左右2輪のモーターを制御します。
 * - PWM出力
 * - 方向制御
 */
#pragma once

#include <Arduino.h>

class MotorController {
public:
    MotorController(
				uint8_t pin_dir_l, 
				uint8_t pin_pwm_l, 
				uint8_t pin_dir_r, 
				uint8_t pin_pwm_r
				);

    // モーター制御を初期化
    // pwm_freq PWM周波数 [Hz]
		// pwm_resolution PWM分解能 [bit]
    void begin(uint32_t pwm_freq = 20000, uint8_t pwm_resolution = 8);

    // PWM値（-255 ~ +255、正=前進、負=後退）
    void setLeftMotor(double pwm);
    void setRightMotor(double pwm);
		 
    void setBothMotors(double pwm_l, double pwm_r);

    // 全モーターを停止
    void stop();

private:
    uint8_t pin_dir_l_;
    uint8_t pin_pwm_l_;
    uint8_t pin_dir_r_;
    uint8_t pin_pwm_r_;
};
