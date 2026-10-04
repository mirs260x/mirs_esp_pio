#pragma once
#include <nav_msgs/msg/odometry.h>

#include "publisher_plugin.hpp"

/** @brief オドメトリ発行（/odom。毎周期。frame: odom -> base_footprint）。
 *  @details 速度は制御時パラメータで確定済みの値を使う（発行時再計算による乖離防止）。 */
class OdomPublisher : public IPublisherPlugin {
public:
    const char *name() const override { return "odom"; }
    bool advertise(rcl_node_t *node) override;
    void publish(const SharedMotion &motion, const SharedSensor &sensor,
                 int32_t now_sec, uint32_t now_nsec) override;
    void release(rcl_node_t *node) override;

private:
    rcl_publisher_t odom_pub_{};
    nav_msgs__msg__Odometry odom_msg_{};
};
