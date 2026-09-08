/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "common/common.h"
#include "estimator.h"
#include "parameters.h"
#include "preprocess.h"
#include <pch.h>

namespace small_point_lio {

    class SmallPointLio {
    private:
        rclcpp::Logger logger;
        Parameters parameters;
        Preprocess preprocess;
        Estimator estimator;
        double time_current = 0.0;
        std::vector<Eigen::Vector3f> pointcloud_odom_frame;
        std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> pointcloud_callback;
        std::function<void(const common::Odometry &odometry)> odometry_callback;
        bool is_init = false;
        std::uint64_t active_map_scan_id = 0;
        std::vector<Eigen::Vector3f> pending_scan_map_points;
        bool planar_reference_initialized = false;
        Eigen::Matrix<state::value_type, 3, 3> planar_tilt_rotation =
                Eigen::Matrix<state::value_type, 3, 3>::Identity();

    public:
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> Q;

        explicit SmallPointLio(rclcpp::Node &node);

        void reset();

        void on_point_cloud_callback(const std::vector<common::Point> &pointcloud);

        void on_imu_callback(const common::ImuMsg &imu_msg);

        void handle_once();

        void set_pointcloud_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> &pointcloud_callback);

        void set_odometry_callback(const std::function<void(const common::Odometry &odometry)> &odometry_callback);

    private:
        void predict_state_with_diagnostics(double timestamp);

        void log_motion_diagnostics();

        void apply_planar_constraint();

        void begin_map_scan(std::uint64_t scan_id);

        void flush_pending_map_points();

        void publish_odometry(double timestamp);
    };

}// namespace small_point_lio
