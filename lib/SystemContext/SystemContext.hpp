#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// タスク間共有の置き場所。tasksとlibの結合点。
// ROSの型（Twist等）は持ち込まない。
// Motion系はcontrol task、Sensor系はsensor taskが書込み、ros taskが読む。
// （1構造体に2 writerを置かない。get-modify-setの消失更新を避けるため）

// ROS速度指令（ros_task → control）。深さ1のmailboxで最新値のみ受渡す
/** @brief ROS速度指令メールボックス要素。 */
struct RosVelocityCmd {
    float linear_x = 0.0f;
    float angular_z = 0.0f;
    uint32_t stamp_ms = 0;
};

// 車体・PIDパラメータ（/params受信→各タスク参照）。mutex保護
/** @brief 車体・PID共有パラメータ。 */
struct SharedParams {
    double wheel_radius = 0.04;
    double wheel_base = 0.38;
    double rkp = 40.0, rki = 150.0, rkd = 0.4;
    double lkp = 40.0, lki = 150.0, lkd = 0.4;

    /** @brief 全フィールド一致判定（パディングの影響を受けない）。
     *  @param other 比較対象
     *  @return 全一致時true
     */
    bool operator==(const SharedParams &other) const {
        return wheel_radius == other.wheel_radius && wheel_base == other.wheel_base &&
               rkp == other.rkp && rki == other.rki && rkd == other.rkd &&
               lkp == other.lkp && lki == other.lki && lkd == other.lkd;
    }
};

// 運動系テレメトリ（control task書込→ros task発行）。mutex保護
/** @brief 運動系テレメトリ。 */
struct SharedMotion {
    int32_t count_l = 0;
    int32_t count_r = 0;
    float vel_l = 0.0f;
    float vel_r = 0.0f;
    float vel_cmd_l = 0.0f;
    float vel_cmd_r = 0.0f;
    uint16_t rc_pulse[3] = {0, 0, 0};
    uint8_t ctrl_mode = 0;  // 0:MANUAL 1:ROS2
    float odom_x = 0.0f;      // オドメトリ推定位置・姿勢（/odomで配信）
    float odom_y = 0.0f;
    float odom_theta = 0.0f;
    float lin_vel = 0.0f;     // 推定直進速度 [m/s]（制御時パラメータで確定済み）
    float ang_vel = 0.0f;     // 推定角速度 [rad/s]（同上）
};

// センサ系テレメトリ（sensor task書込→ros task発行）。mutex保護
/** @brief センサ系テレメトリ。 */
struct SharedSensor {
    float voltage_1 = 0.0f;
    float voltage_2 = 0.0f;
    bool imu_ok = false;
    bool mag_ok = false;
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float mx = 0.0f, my = 0.0f, mz = 0.0f;
};

/** @brief mutex保護の共有値。set/getの定型を集約する。 */
template <typename T>
class SharedValue {
public:
    /** @brief mutex生成。 */
    void create() {
        if (mutex_ == nullptr) {
            mutex_ = xSemaphoreCreateMutex();
        }
    }

    /** @brief 書き込む。mutex取得失敗時は捨てる。
     *  @param value 書き込む値
     */
    void set(const T &value) {
        if (mutex_ == nullptr) {
            value_ = value;
            return;
        }
        if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
            value_ = value;
            xSemaphoreGive(mutex_);
        }
    }

    /** @brief 読み出す。mutex取得失敗時は前回成功値を返す（破損値を読まない）。
     *  @return スナップショット
     */
    T get() const {
        if (mutex_ == nullptr) {
            return value_;
        }
        if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
            cache_ = value_;
            xSemaphoreGive(mutex_);
        }
        return cache_;
    }

private:
    T value_{};
    mutable T cache_{};
    mutable SemaphoreHandle_t mutex_ = nullptr;
};

/** @brief タスク間共有ストア。 */
class SystemContext {
public:
    /** @brief キュー・mutex生成。 */
    void begin();

    // ROS指令キュー（非ブロッキング）
    bool pushRosCmd(const RosVelocityCmd &cmd);
    bool popRosCmd(RosVelocityCmd &cmd);

    // パラメータ・テレメトリ（mutex保護コピー）
    void setParams(const SharedParams &p) { params_.set(p); }
    SharedParams getParams() { return params_.get(); }
    void setMotion(const SharedMotion &m) { motion_.set(m); }
    SharedMotion getMotion() { return motion_.get(); }
    void setSensor(const SharedSensor &s) { sensor_.set(s); }
    SharedSensor getSensor() { return sensor_.get(); }

private:
    QueueHandle_t ros_cmd_queue_ = nullptr;
    SharedValue<SharedParams> params_;
    SharedValue<SharedMotion> motion_;
    SharedValue<SharedSensor> sensor_;
};

extern SystemContext g_sys;
