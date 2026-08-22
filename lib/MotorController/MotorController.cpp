/**
 * @file MotorController.cpp
 * @brief MotorControllerクラスの実装
 */

#include "MotorController.hpp"

MotorController::MotorController(uint8_t pin_dir_l, uint8_t pin_pwm_l, uint8_t pin_dir_r, uint8_t pin_pwm_r)
    : pin_dir_l_(pin_dir_l)
    , pin_pwm_l_(pin_pwm_l)
    , pin_dir_r_(pin_dir_r)
    , pin_pwm_r_(pin_pwm_r)
{
}

void MotorController::begin(uint32_t pwm_freq, uint8_t pwm_resolution) {
    // ピンモード設定
    pinMode(pin_dir_r_, OUTPUT);
    pinMode(pin_dir_l_, OUTPUT);
    
    // PWM設定
    ledcAttach(pin_pwm_r_, pwm_freq, pwm_resolution);
    ledcAttach(pin_pwm_l_, pwm_freq, pwm_resolution);
    
    // 初期状態：停止
    stop();
}

void MotorController::setLeftMotor(double pwm) {
    // 方向設定（正=HIGH、負=LOW）
    digitalWrite(pin_dir_l_, pwm >= 0 ? HIGH : LOW);
    
    // PWM出力（絶対値）
    ledcWrite(pin_pwm_l_, (uint8_t)abs(pwm));
}

void MotorController::setRightMotor(double pwm) {
    // 方向設定（正=LOW、負=HIGH：右モーターは逆）
    digitalWrite(pin_dir_r_, pwm >= 0 ? LOW : HIGH);
    
    // PWM出力（絶対値）
    ledcWrite(pin_pwm_r_, (uint8_t)abs(pwm));
}

void MotorController::setBothMotors(double pwm_l, double pwm_r) {
    setLeftMotor(pwm_l);
    setRightMotor(pwm_r);
}

void MotorController::stop() {
    ledcWrite(pin_pwm_r_, 0);
    ledcWrite(pin_pwm_l_, 0);
}
