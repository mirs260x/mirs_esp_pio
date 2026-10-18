#include "telemetry_publisher.hpp"

#include <rclc/rclc.h>

#include <cstdlib>

bool TelemetryPublisher::allocBuffers() {
    enc_msg_.data.capacity = 2;
    enc_msg_.data.size = 2;
    enc_msg_.data.data = (int32_t *)malloc(2 * sizeof(int32_t));
    vel_msg_.data.capacity = 2;
    vel_msg_.data.size = 2;
    vel_msg_.data.data = (double *)malloc(2 * sizeof(double));
    vlt_msg_.data.capacity = 2;
    vlt_msg_.data.size = 2;
    vlt_msg_.data.data = (double *)malloc(2 * sizeof(double));
    rc_debug_msg_.data.capacity = 6;
    rc_debug_msg_.data.size = 6;
    rc_debug_msg_.data.data = (double *)malloc(6 * sizeof(double));

    if (enc_msg_.data.data == nullptr || vel_msg_.data.data == nullptr ||
        vlt_msg_.data.data == nullptr || rc_debug_msg_.data.data == nullptr) {
        freeBuffers();
        return false;
    }
    enc_msg_.data.data[0] = enc_msg_.data.data[1] = 0;
    vel_msg_.data.data[0] = vel_msg_.data.data[1] = 0.0;
    vlt_msg_.data.data[0] = vlt_msg_.data.data[1] = 0.0;
    for (int i = 0; i < 6; i++) rc_debug_msg_.data.data[i] = 0.0;
    return true;
}

void TelemetryPublisher::freeBuffers() {
    free(enc_msg_.data.data);
    free(vel_msg_.data.data);
    free(vlt_msg_.data.data);
    free(rc_debug_msg_.data.data);
    enc_msg_.data.data = nullptr;
    vel_msg_.data.data = nullptr;
    vlt_msg_.data.data = nullptr;
    rc_debug_msg_.data.data = nullptr;
}

bool TelemetryPublisher::advertise(rcl_node_t *node) {
    if (!allocBuffers()) {
        return false;
    }
    if (rclc_publisher_init_default(&enc_pub_, node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
                                    "/encoder") != RCL_RET_OK ||
        rclc_publisher_init_default(&vel_pub_, node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
                                    "/vel") != RCL_RET_OK ||
        rclc_publisher_init_default(&vlt_pub_, node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
                                    "/vlt") != RCL_RET_OK ||
        rclc_publisher_init_default(&rc_debug_pub_, node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
                                    "/rc_debug") != RCL_RET_OK) {
        release(node);
        return false;
    }
    return true;
}

void TelemetryPublisher::publish(const SharedMotion &motion, const SharedSensor &sensor,
                                 int32_t /*now_sec*/, uint32_t /*now_nsec*/) {
    enc_msg_.data.data[0] = motion.count_l;
    enc_msg_.data.data[1] = motion.count_r;
    vel_msg_.data.data[0] = motion.vel_l;
    vel_msg_.data.data[1] = motion.vel_r;

    if (!divider_.tick()) {
        return;
    }
    vlt_msg_.data.data[0] = sensor.voltage_1;
    vlt_msg_.data.data[1] = sensor.voltage_2;

    // RC デバッグ情報: [0:ChLeft_us, 1:ChMode_us, 2:ChRight_us, 3:l_vel_cmd, 4:r_vel_cmd, 5:Mode(0:Manual,1:ROS2)]
    rc_debug_msg_.data.data[0] = (double)motion.rc_pulse[0];
    rc_debug_msg_.data.data[1] = (double)motion.rc_pulse[1];
    rc_debug_msg_.data.data[2] = (double)motion.rc_pulse[2];
    rc_debug_msg_.data.data[3] = motion.vel_cmd_l;
    rc_debug_msg_.data.data[4] = motion.vel_cmd_r;
    rc_debug_msg_.data.data[5] = (double)motion.ctrl_mode;

    ignoreResult(rcl_publish(&enc_pub_, &enc_msg_, NULL));
    ignoreResult(rcl_publish(&vel_pub_, &vel_msg_, NULL));
    ignoreResult(rcl_publish(&vlt_pub_, &vlt_msg_, NULL));
    ignoreResult(rcl_publish(&rc_debug_pub_, &rc_debug_msg_, NULL));
}

void TelemetryPublisher::release(rcl_node_t *node) {
    ignoreResult(rcl_publisher_fini(&enc_pub_, node));
    ignoreResult(rcl_publisher_fini(&vel_pub_, node));
    ignoreResult(rcl_publisher_fini(&vlt_pub_, node));
    ignoreResult(rcl_publisher_fini(&rc_debug_pub_, node));
    freeBuffers();
}
