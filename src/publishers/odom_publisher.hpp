#pragma once
#include <nav_msgs/msg/odometry.h>

#include "publish_divider.hpp"
#include "publisher_plugin.hpp"

/** @brief オドメトリ発行（/odom。15ms周期の4回に1回＝約17Hz。frame: odom -> base_footprint）。
 *  @details 速度は制御時パラメータで確定済みの値を使う（発行時再計算による乖離防止）。
 *  115200bps半二重シリアルを飽和させないため毎周期発行しない。 */
class OdomPublisher : public IPublisherPlugin {
public:
    const char *name() const override { return "odom"; }
    bool advertise(rcl_node_t *node) override;
    void publish(const SharedMotion &motion, const SharedSensor &sensor,
                 int32_t now_sec, uint32_t now_nsec) override;
    void release(rcl_node_t *node) override;

private:
    static constexpr uint8_t kPublishDivider = 4;  // [回] 4周期に1回発行
    rcl_publisher_t odom_pub_{};
    nav_msgs__msg__Odometry odom_msg_{};
    PublishDivider divider_{kPublishDivider};
};
