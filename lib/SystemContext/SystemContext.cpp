#include "SystemContext.hpp"

SystemContext g_sys;

void SystemContext::begin() {
    ros_cmd_queue_ = xQueueCreate(4, sizeof(RosVelocityCmd));
    param_mutex_ = xSemaphoreCreateMutex();
    motion_mutex_ = xSemaphoreCreateMutex();
    sensor_mutex_ = xSemaphoreCreateMutex();
}

bool SystemContext::pushRosCmd(const RosVelocityCmd &cmd) {
    if (ros_cmd_queue_ == nullptr) {
        return false;
    }
    return xQueueSend(ros_cmd_queue_, &cmd, 0) == pdTRUE;
}

bool SystemContext::popRosCmd(RosVelocityCmd &cmd) {
    if (ros_cmd_queue_ == nullptr) {
        return false;
    }
    return xQueueReceive(ros_cmd_queue_, &cmd, 0) == pdTRUE;
}

void SystemContext::setParams(const SharedParams &p) {
    if (xSemaphoreTake(param_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        params_ = p;
        xSemaphoreGive(param_mutex_);
    }
}

SharedParams SystemContext::getParams() {
    SharedParams p{};
    if (xSemaphoreTake(param_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        p = params_;
        xSemaphoreGive(param_mutex_);
    }
    return p;
}

void SystemContext::setMotion(const SharedMotion &m) {
    if (xSemaphoreTake(motion_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        motion_ = m;
        xSemaphoreGive(motion_mutex_);
    }
}

SharedMotion SystemContext::getMotion() {
    SharedMotion m{};
    if (xSemaphoreTake(motion_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        m = motion_;
        xSemaphoreGive(motion_mutex_);
    }
    return m;
}

void SystemContext::setSensor(const SharedSensor &s) {
    if (xSemaphoreTake(sensor_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        sensor_ = s;
        xSemaphoreGive(sensor_mutex_);
    }
}

SharedSensor SystemContext::getSensor() {
    SharedSensor s{};
    if (xSemaphoreTake(sensor_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        s = sensor_;
        xSemaphoreGive(sensor_mutex_);
    }
    return s;
}
