#include "SystemContext.hpp"

SystemContext g_sys;

void SystemContext::begin() {
    ros_cmd_queue_ = xQueueCreate(4, sizeof(RosVelocityCmd));
    param_mutex_ = xSemaphoreCreateMutex();
    telemetry_mutex_ = xSemaphoreCreateMutex();
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

void SystemContext::setTelemetry(const SharedTelemetry &t) {
    if (xSemaphoreTake(telemetry_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        telemetry_ = t;
        xSemaphoreGive(telemetry_mutex_);
    }
}

SharedTelemetry SystemContext::getTelemetry() {
    SharedTelemetry t{};
    if (xSemaphoreTake(telemetry_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        t = telemetry_;
        xSemaphoreGive(telemetry_mutex_);
    }
    return t;
}
