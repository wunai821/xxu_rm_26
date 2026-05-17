/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "base_lidar.h"
#include <cstring>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace small_point_lio {

    class GenericPointCloud2Adapter : public LidarAdapterBase {
    private:
        struct FieldInfo {
            uint32_t offset = 0;
            uint8_t datatype = 0;
            bool exists = false;
        };

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;

        static FieldInfo find_field(const sensor_msgs::msg::PointCloud2 &msg, const std::string &name) {
            for (const auto &field: msg.fields) {
                if (field.name == name && field.count == 1) {
                    return FieldInfo{field.offset, field.datatype, true};
                }
            }
            return {};
        }

        static bool read_float32(const sensor_msgs::msg::PointCloud2 &msg, size_t point_offset, const FieldInfo &field, float &value) {
            if (!field.exists || field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
                return false;
            }
            if (point_offset + field.offset + sizeof(float) > msg.data.size()) {
                return false;
            }
            std::memcpy(&value, msg.data.data() + point_offset + field.offset, sizeof(float));
            return std::isfinite(value);
        }

        static bool read_timestamp(const sensor_msgs::msg::PointCloud2 &msg, size_t point_offset, const FieldInfo &field, double &value) {
            if (!field.exists) {
                return false;
            }
            const size_t offset = point_offset + field.offset;
            if (field.datatype == sensor_msgs::msg::PointField::FLOAT64) {
                if (offset + sizeof(double) > msg.data.size()) {
                    return false;
                }
                std::memcpy(&value, msg.data.data() + offset, sizeof(double));
                return std::isfinite(value);
            }
            if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
                if (offset + sizeof(float) > msg.data.size()) {
                    return false;
                }
                float timestamp = 0.0F;
                std::memcpy(&timestamp, msg.data.data() + offset, sizeof(float));
                value = timestamp;
                return std::isfinite(value);
            }
            if (field.datatype == sensor_msgs::msg::PointField::UINT32) {
                if (offset + sizeof(uint32_t) > msg.data.size()) {
                    return false;
                }
                uint32_t timestamp = 0;
                std::memcpy(&timestamp, msg.data.data() + offset, sizeof(uint32_t));
                value = static_cast<double>(timestamp) * 1e-9;
                return true;
            }
            return false;
        }

    public:
        inline void setup_subscription(rclcpp::Node *node, const std::string &topic, std::function<void(const std::vector<common::Point> &)> callback) override {
            const double scan_period = node->declare_parameter<double>("scan_period", 0.1);
            subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
                    topic,
                    rclcpp::SensorDataQoS(),
                    [callback, scan_period, logger = node->get_logger(), clock = node->get_clock()](const sensor_msgs::msg::PointCloud2 &msg) {
                        const auto x_field = find_field(msg, "x");
                        const auto y_field = find_field(msg, "y");
                        const auto z_field = find_field(msg, "z");
                        const auto timestamp_field = find_field(msg, "timestamp");
                        if (!x_field.exists || !y_field.exists || !z_field.exists) {
                            RCLCPP_WARN_THROTTLE(logger, *clock, 2000, "PointCloud2 x/y/z fields are required");
                            return;
                        }

                        const size_t size = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
                        const double stamp = static_cast<double>(msg.header.stamp.sec) + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
                        std::vector<common::Point> pointcloud;
                        pointcloud.reserve(size);

                        for (size_t i = 0; i < size; ++i) {
                            const size_t point_offset = i * msg.point_step;
                            float x = 0.0F;
                            float y = 0.0F;
                            float z = 0.0F;
                            if (!read_float32(msg, point_offset, x_field, x) ||
                                !read_float32(msg, point_offset, y_field, y) ||
                                !read_float32(msg, point_offset, z_field, z)) {
                                continue;
                            }

                            double timestamp = 0.0;
                            if (!read_timestamp(msg, point_offset, timestamp_field, timestamp)) {
                                const double ratio = size > 1 ? static_cast<double>(i) / static_cast<double>(size - 1) : 0.0;
                                timestamp = stamp + scan_period * ratio;
                            }

                            common::Point new_point;
                            new_point.position << x, y, z;
                            new_point.timestamp = timestamp;
                            pointcloud.push_back(new_point);
                        }
                        callback(pointcloud);
                    });
        }
    };

}// namespace small_point_lio
