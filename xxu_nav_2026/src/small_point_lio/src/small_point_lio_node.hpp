/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "common/common.h"
#include "lidar_adapter/base_lidar.h"
#include "small_point_lio/small_point_lio.h"
#include "util/pointcloud_mapping.h"
#include <nav_msgs/msg/odometry.hpp>
#include <pch.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <deque>
#include <mutex>

namespace small_point_lio {

    class SmallPointLioNode : public rclcpp::Node {
    private:
        struct PendingDeskewScan {
            std::vector<common::Point> points;
            double start_timestamp = 0.0;
            double end_timestamp = 0.0;
            std::chrono::steady_clock::time_point queued_at;
        };

        std::unique_ptr<small_point_lio::SmallPointLio> small_point_lio;
        std::unique_ptr<LidarAdapterBase> lidar_adapter;
        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> imu_subsciber;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloud_publisher;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_trigger;
        std::unique_ptr<util::PointcloudMapping> pointcloud_mapping;
        std::string lidar_frame;
        std::string base_frame;
        double tf_lookup_timeout = 0.1;
        double deskew_pending_timeout = 0.25;
        double deskew_pose_history_duration = 5.0;
        std::size_t deskew_pending_queue_size = 3;
        std::deque<common::Odometry> pose_history;
        std::deque<PendingDeskewScan> pending_deskew_scans;
        std::mutex deskew_mutex;
        std::mutex deskew_process_mutex;
        rclcpp::TimerBase::SharedPtr deskew_retry_timer;
        bool has_last_published_pose = false;
        double last_published_pose_timestamp = 0.0;
        Eigen::Vector3d last_published_position = Eigen::Vector3d::Zero();
        Eigen::Quaterniond last_published_orientation = Eigen::Quaterniond::Identity();

    public:
        explicit SmallPointLioNode(const rclcpp::NodeOptions &options);

    private:
        void on_pose_history(const common::Odometry &odometry);

        void on_dense_scan(const std::vector<common::Point> &pointcloud);

        void process_pending_deskew_scans();
    };

}// namespace small_point_lio
