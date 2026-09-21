/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "base_lidar.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "validated_livox_cloud.h"

namespace small_point_lio {

    class LivoxPointCloud2Adapter : public LidarAdapterBase {
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;

    public:
        inline void setup_subscription(rclcpp::Node *node, const std::string &topic, std::function<void(const std::vector<common::Point> &)> callback) override {
            subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
                    topic,
                    rclcpp::SensorDataQoS(),
                    [callback, logger = node->get_logger(), clock = node->get_clock()](const sensor_msgs::msg::PointCloud2 &msg) {
                        std::array<uint32_t, 5> offsets{};
                        if (!validateLivoxCloud(msg, offsets)) {
                            RCLCPP_WARN_THROTTLE(logger, *clock, 2000, "Rejecting malformed Livox PointCloud2");
                            return;
                        }
                        const size_t size = size_t(msg.width) * msg.height;
                        std::vector<common::Point> pointcloud;
                        pointcloud.reserve(size);
                        for (uint32_t row = 0; row < msg.height; ++row) {
                          for (uint32_t col = 0; col < msg.width; ++col) {
                            const auto *data = msg.data.data() + size_t(row) * msg.row_step + size_t(col) * msg.point_step;
                            if ((data[offsets[3]] & 0b00111111) == 0b00000000) {
                                common::Point new_point;
                                new_point.position << readLivoxValue<float>(data + offsets[0], msg.is_bigendian),
                                    readLivoxValue<float>(data + offsets[1], msg.is_bigendian),
                                    readLivoxValue<float>(data + offsets[2], msg.is_bigendian);
                                new_point.timestamp = readLivoxValue<double>(data + offsets[4], msg.is_bigendian) * 1e-9;
                                if (new_point.position.allFinite() && std::isfinite(new_point.timestamp)) {
                                    pointcloud.push_back(new_point);
                                }
                            }
                          }
                        }
                        callback(pointcloud);
                    });
        }
    };

}// namespace small_point_lio
