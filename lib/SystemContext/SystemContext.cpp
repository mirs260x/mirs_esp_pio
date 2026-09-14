#include "SystemContext.hpp"

SystemContext g_sys;

void SystemContext::begin() {
    // 二重beginのリーク防止。main setup()から1回のみ想定だが安全側に倒す
    if (ros_cmd_queue_ == nullptr) {
        ros_cmd_queue_ = xQueueCreate(1, sizeof(RosVelocityCmd));  // 最新値のみ保持のmailbox
    }
    if (param_mutex_ == nullptr) {
        param_mutex_ = xSemaphoreCreateMutex();
    }
    if (motion_mutex_ == nullptr) {
        motion_mutex_ = xSemaphoreCreateMutex();
    }
    if (sensor_mutex_ == nullptr) {
        sensor_mutex_ = xSemaphoreCreateMutex();
    }
}

bool SystemContext::pushRosCmd(const RosVelocityCmd &cmd) {
    if (ros_cmd_queue_ == nullptr) {
        return false;
    }
    // mailbox運用：常に最新値で上書きする。溢れ時の黙殺は発生しない
    return xQueueOverwrite(ros_cmd_queue_, &cmd) == pdTRUE;
}

bool SystemContext::popRosCmd(RosVelocityCmd &cmd) {
    if (ros_cmd_queue_ == nullptr) {
        return false;
    }
    return xQueueReceive(ros_cmd_queue_, &cmd, 0) == pdTRUE;
}

void SystemContext::setParams(const SharedParams &p) {
    // begin()前は単一タスクのみ動作中のため無保護で代入し、フォールバック値を維持する
    if (param_mutex_ == nullptr) {
        params_ = p;
        return;
    }
    if (xSemaphoreTake(param_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        params_ = p;
        xSemaphoreGive(param_mutex_);
    }
}

SharedParams SystemContext::getParams() {
    // begin()前はフォールバック既定値(params_初期値)をそのまま返す。ゼロ値化しない
    if (param_mutex_ == nullptr) {
        return params_;
    }
    SharedParams p = params_;  // タイムアウト時は黙ってゼロを返さず直近値を返す
    if (xSemaphoreTake(param_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        p = params_;
        xSemaphoreGive(param_mutex_);
    }
    return p;
}

void SystemContext::setMotion(const SharedMotion &m) {
    if (motion_mutex_ == nullptr) {
        motion_ = m;
        return;
    }
    if (xSemaphoreTake(motion_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        motion_ = m;
        xSemaphoreGive(motion_mutex_);
    }
}

SharedMotion SystemContext::getMotion() {
    if (motion_mutex_ == nullptr) {
        return motion_;
    }
    SharedMotion m = motion_;
    if (xSemaphoreTake(motion_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        m = motion_;
        xSemaphoreGive(motion_mutex_);
    }
    return m;
}

void SystemContext::setSensor(const SharedSensor &s) {
    if (sensor_mutex_ == nullptr) {
        sensor_ = s;
        return;
    }
    if (xSemaphoreTake(sensor_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        sensor_ = s;
        xSemaphoreGive(sensor_mutex_);
    }
}

SharedSensor SystemContext::getSensor() {
    if (sensor_mutex_ == nullptr) {
        return sensor_;
    }
    SharedSensor s = sensor_;
    if (xSemaphoreTake(sensor_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        s = sensor_;
        xSemaphoreGive(sensor_mutex_);
    }
    return s;
}
