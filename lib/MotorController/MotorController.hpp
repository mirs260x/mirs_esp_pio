/**
 * @file MotorController.hpp
 * @brief モーター制御クラス
 * 
 * 左右2輪のモーターを制御します。
 * - PWM出力
 * - 方向制御
 */

#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <Arduino.h>

class MotorController {
public:
    /**
     * @brief コンストラクタ
     * @param pin_dir_l 左モーター方向ピン
     * @param pin_pwm_l 左モーターPWMピン
     * @param pin_dir_r 右モーター方向ピン
     * @param pin_pwm_r 右モーターPWMピン
     */
    MotorController(uint8_t pin_dir_l, uint8_t pin_pwm_l, uint8_t pin_dir_r, uint8_t pin_pwm_r);

    /**
     * @brief モーター制御を初期化
     * @param pwm_freq PWM周波数 [Hz]
     * @param pwm_resolution PWM分解能 [bit]
     */
    void begin(uint32_t pwm_freq = 20000, uint8_t pwm_resolution = 8);

    /**
     * @brief 左モーターを制御
     * @param pwm PWM値（-255 ~ +255、正=前進、負=後退）
     */
    void setLeftMotor(double pwm);

    /**
     * @brief 右モーターを制御
     * @param pwm PWM値（-255 ~ +255、正=前進、負=後退）
     */
    void setRightMotor(double pwm);

    /**
     * @brief 左右モーターを同時制御
     * @param pwm_l 左モーターPWM値
     * @param pwm_r 右モーターPWM値
     */
    void setBothMotors(double pwm_l, double pwm_r);

    /**
     * @brief 全モーターを停止
     */
    void stop();

private:
    uint8_t pin_dir_l_;  ///< 左モーター方向ピン
    uint8_t pin_pwm_l_;  ///< 左モーターPWMピン
    uint8_t pin_dir_r_;  ///< 右モーター方向ピン
    uint8_t pin_pwm_r_;  ///< 右モーターPWMピン
};

#endif // MOTOR_CONTROLLER_HPP
