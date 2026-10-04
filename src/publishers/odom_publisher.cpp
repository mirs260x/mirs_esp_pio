#include "odom_publisher.hpp"

#include <rclc/rclc.h>

#include <cmath>
#include <cstring>

bool OdomPublisher::advertise(rcl_node_t *node) {
    odom_msg_.header.frame_id.data = (char *)"odom";
    odom_msg_.header.frame_id.size = strlen("odom");
    odom_msg_.header.frame_id.capacity = odom_msg_.header.frame_id.size + 1;
    odom_msg_.child_frame_id.data = (char *)"base_footprint";
    odom_msg_.child_frame_id.size = strlen("base_footprint");
    odom_msg_.child_frame_id.capacity = odom_msg_.child_frame_id.size + 1;

    // pose / twist covariance: diagonal (yawのみ0.05。rough estimate)
    constexpr double kCovDiag = 0.01;
    constexpr double kCovYaw = 0.05;
    auto set_diag6 = [](double *cov) {
        cov[0] = kCovDiag;
        cov[7] = kCovDiag;
        cov[14] = kCovDiag;
        cov[21] = kCovDiag;
        cov[28] = kCovDiag;
        cov[35] = kCovDiag;
    };
    set_diag6(odom_msg_.pose.covariance);
    odom_msg_.pose.covariance[14] = kCovYaw;
    set_diag6(odom_msg_.twist.covariance);

    return rclc_publisher_init_default(&odom_pub_, node,
                                       ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                                       "/odom") == RCL_RET_OK;
}

void OdomPublisher::publish(const SharedMotion &motion, const SharedSensor & /*sensor*/,
                            int32_t now_sec, uint32_t now_nsec) {
    const double half_yaw = motion.odom_theta * 0.5;

    odom_msg_.header.stamp.sec = now_sec;
    odom_msg_.header.stamp.nanosec = now_nsec;
    odom_msg_.pose.pose.position.x = motion.odom_x;
    odom_msg_.pose.pose.position.y = motion.odom_y;
    odom_msg_.pose.pose.orientation.z = std::sin(half_yaw);
    odom_msg_.pose.pose.orientation.w = std::cos(half_yaw);
    odom_msg_.twist.twist.linear.x = motion.lin_vel;
    odom_msg_.twist.twist.angular.z = motion.ang_vel;

    ignoreResult(rcl_publish(&odom_pub_, &odom_msg_, NULL));
}

void OdomPublisher::release(rcl_node_t *node) {
    ignoreResult(rcl_publisher_fini(&odom_pub_, node));
}
