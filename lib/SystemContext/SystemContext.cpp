#include "SystemContext.hpp"

SystemContext g_sys;

void SystemContext::begin() {
    // 二重beginのリーク防止。main setup()から1回のみ想定だが安全側に倒す
    if (ros_cmd_queue_ == nullptr) {
        ros_cmd_queue_ = xQueueCreate(1, sizeof(RosVelocityCmd));  // 最新値のみ保持のmailbox
    }
    params_.create();
    motion_.create();
    sensor_.create();
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
