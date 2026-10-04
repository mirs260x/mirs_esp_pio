#pragma once
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

#include "publisher_plugin.hpp"

/** @brief IMU発行（/imu/data_raw・/imu/mag。毎周期）。
 *  @details データ不在時（imu_ok/mag_ok=false）は無発行にする。#if分離はしない。 */
class ImuPublisher : public IPublisherPlugin {
public:
    const char *name() const override { return "imu"; }
    bool advertise(rcl_node_t *node) override;
    void publish(const SharedMotion &motion, const SharedSensor &sensor,
                 int32_t now_sec, uint32_t now_nsec) override;
    void release(rcl_node_t *node) override;

private:
    rcl_publisher_t imu_pub_{};
    rcl_publisher_t mag_pub_{};
    sensor_msgs__msg__Imu imu_msg_{};
    sensor_msgs__msg__MagneticField mag_msg_{};
};
