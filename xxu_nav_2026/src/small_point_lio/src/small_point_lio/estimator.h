/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "eskf.h"
#include "parameters.h"
#include "small_ivox.h"
#include <pch.h>

namespace small_point_lio {

    class Estimator {
    public:
        // Accumulated only when motion_diagnostics_en is enabled.
        struct MotionDiagnostics {
            double start = 0.0;
            Eigen::Vector3d position_start = Eigen::Vector3d::Zero();
            Eigen::Vector3d predict_dp = Eigen::Vector3d::Zero();
            Eigen::Vector3d point_dp = Eigen::Vector3d::Zero();
            Eigen::Vector3d imu_dp = Eigen::Vector3d::Zero();
            Eigen::Vector3d constraint_dp = Eigen::Vector3d::Zero();
            Eigen::Vector3d predict_dv = Eigen::Vector3d::Zero();
            Eigen::Vector3d point_dv = Eigen::Vector3d::Zero();
            Eigen::Vector3d imu_dv = Eigen::Vector3d::Zero();
            Eigen::Matrix2d normal_xy_sum = Eigen::Matrix2d::Zero();
            double residual_squared_sum = 0.0;
            std::size_t attempted = 0, accepted = 0;
            std::size_t no_neighbors = 0, nonplanar = 0, residual_rejected = 0;
            std::size_t imu_updates = 0, imu_failed = 0;
            std::size_t late_points = 0, late_imus = 0;
        } diagnostics;

        // for common
        Parameters *parameters = nullptr;
        eskf kf;
        // for h_point
        std::shared_ptr<SmallIVox> ivox;
        Eigen::Matrix<state::value_type, 3, 1> Lidar_T_wrt_IMU;
        Eigen::Matrix<state::value_type, 3, 3> Lidar_R_wrt_IMU;
        Eigen::Vector3f point_lidar_frame;
        Eigen::Vector3f point_odom_frame;
        std::vector<Eigen::Vector3f> nearest_points;
        // for h_imu
        Eigen::Matrix<state::value_type, 3, 1> angular_velocity;
        Eigen::Matrix<state::value_type, 3, 1> linear_acceleration;
        double imu_acceleration_scale;

        Estimator();

        void reset();

        [[nodiscard]] Eigen::Matrix<state::value_type, state::DIM, state::DIM> process_noise_cov() const;

        void h_point(const state &s, point_measurement_result &measurement_result);

        void h_imu(const state &s, imu_measurement_result &measurement_result);
    };

}// namespace small_point_lio
