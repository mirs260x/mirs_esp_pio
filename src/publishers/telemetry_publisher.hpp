#pragma once
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>

#include "publisher_plugin.hpp"

/** @brief 低頻度テレメトリ一括発行（/encoder・/vel・/vlt・/rc_debug）。
 *  @details 呼ばれ方は毎周期だが、PUBLISH_DIVIDER回に1回だけ発行する。 */
class TelemetryPublisher : public IPublisherPlugin {
public:
    const char *name() const override { return "telemetry"; }
    bool advertise(rcl_node_t *node) override;
    void publish(const SharedMotion &motion, const SharedSensor &sensor,
                 int32_t now_sec, uint32_t now_nsec) override;
    void release(rcl_node_t *node) override;

private:
    bool allocBuffers();
    void freeBuffers();

    rcl_publisher_t enc_pub_{};
    rcl_publisher_t vel_pub_{};
    rcl_publisher_t vlt_pub_{};
    rcl_publisher_t rc_debug_pub_{};
    std_msgs__msg__Int32MultiArray enc_msg_{};
    std_msgs__msg__Float64MultiArray vel_msg_{};
    std_msgs__msg__Float64MultiArray vlt_msg_{};
    std_msgs__msg__Float64MultiArray rc_debug_msg_{};
    uint8_t div_cnt_ = 0;
};
