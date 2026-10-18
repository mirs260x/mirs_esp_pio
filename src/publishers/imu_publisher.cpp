#include "imu_publisher.hpp"

#include <rclc/rclc.h>

#include <cstring>

bool ImuPublisher::advertise(rcl_node_t *node) {
    imu_msg_.header.frame_id.data = (char *)"imu_link";
    imu_msg_.header.frame_id.size = strlen("imu_link");
    imu_msg_.header.frame_id.capacity = imu_msg_.header.frame_id.size + 1;

    // orientationは機上で推定しないため未知扱い（covariance[0]=-1）。
    // 四元数は無効値(0,0,0,0)を避けて単位四元子で初期化する（他consumer対策）
    imu_msg_.orientation.w = 1.0;
    // covariance diagonals are rough estimates (unit: sensor spec dependent)
    constexpr double kCovDiag = 0.01;
    auto set_diag3 = [](double *cov) {
        cov[0] = kCovDiag;
        cov[4] = kCovDiag;
        cov[8] = kCovDiag;
    };
    imu_msg_.orientation_covariance[0] = -1.0;
    set_diag3(imu_msg_.angular_velocity_covariance);
    set_diag3(imu_msg_.linear_acceleration_covariance);

    mag_msg_.header.frame_id.data = (char *)"imu_link";
    mag_msg_.header.frame_id.size = strlen("imu_link");
    mag_msg_.header.frame_id.capacity = mag_msg_.header.frame_id.size + 1;

    // magnetic_field_covariance: diagonal (rough estimate)
    set_diag3(mag_msg_.magnetic_field_covariance);

    if (rclc_publisher_init_default(&imu_pub_, node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                    "/imu/data_raw") != RCL_RET_OK ||
        rclc_publisher_init_default(&mag_pub_, node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
                                    "/imu/mag") != RCL_RET_OK) {
        release(node);
        return false;
    }
    return true;
}

void ImuPublisher::publish(const SharedMotion & /*motion*/, const SharedSensor &sensor,
                           int32_t now_sec, uint32_t now_nsec) {
    // IMU (BMX055) は15ms周期の4回に1回（約17Hz）パブリッシュする
    if (!divider_.tick()) {
        return;
    }
    if (!sensor.imu_ok && !sensor.mag_ok) {
        return;
    }
    if (sensor.imu_ok) {
        imu_msg_.header.stamp.sec = now_sec;
        imu_msg_.header.stamp.nanosec = now_nsec;

        imu_msg_.linear_acceleration.x = sensor.ax;
        imu_msg_.linear_acceleration.y = sensor.ay;
        imu_msg_.linear_acceleration.z = sensor.az;

        imu_msg_.angular_velocity.x = sensor.gx;
        imu_msg_.angular_velocity.y = sensor.gy;
        imu_msg_.angular_velocity.z = sensor.gz;

        ignoreResult(rcl_publish(&imu_pub_, &imu_msg_, NULL));
    }

    if (sensor.mag_ok) {
        mag_msg_.header.stamp.sec = now_sec;
        mag_msg_.header.stamp.nanosec = now_nsec;

        mag_msg_.magnetic_field.x = sensor.mx * 1e-6f;  // uT -> Tesla
        mag_msg_.magnetic_field.y = sensor.my * 1e-6f;
        mag_msg_.magnetic_field.z = sensor.mz * 1e-6f;

        ignoreResult(rcl_publish(&mag_pub_, &mag_msg_, NULL));
    }
}

void ImuPublisher::release(rcl_node_t *node) {
    ignoreResult(rcl_publisher_fini(&imu_pub_, node));
    ignoreResult(rcl_publisher_fini(&mag_pub_, node));
}
