#pragma once
#include "SensorPlugin.hpp"

class BMX055;
struct BMX055Data;

/** @brief IMU（BMX055）プラグイン。加速度・角速度・地磁気を書き込む。
 *  @details 読み取り失敗時は担当フィールドをゼロ化する（前回値の残留を避ける）。
 */
class ImuPlugin : public ISensorPlugin {
public:
    /** @brief コンストラクタ。バス固有値は全て注入する。
     *  @param imu 実体（寿命は呼び出し側が保証すること）
     *  @param sda_pin SDA GPIO番号
     *  @param scl_pin SCL GPIO番号
     */
    ImuPlugin(BMX055 &imu, int sda_pin, int scl_pin);

    const char *name() const override { return "imu"; }
    bool begin() override;
    void update(SharedSensor &snapshot) override;

private:
    BMX055 &imu_;
    int sda_pin_;
    int scl_pin_;
};
