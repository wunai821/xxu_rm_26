/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio.h"

namespace small_point_lio {

    SmallPointLio::SmallPointLio(rclcpp::Node &node) {
        // init param
        parameters.read_parameters(node);
        preprocess.parameters = &parameters;
        estimator.parameters = &parameters;
        estimator.Lidar_T_wrt_IMU = parameters.extrinsic_T.cast<state::value_type>();
        estimator.Lidar_R_wrt_IMU = parameters.extrinsic_R.cast<state::value_type>();
        if (parameters.extrinsic_est_en) {
            estimator.kf.x.offset_T_L_I = parameters.extrinsic_T.cast<state::value_type>();
            estimator.kf.x.offset_R_L_I = parameters.extrinsic_R.cast<state::value_type>();
        }
        Q = estimator.process_noise_cov();
        estimator.imu_acceleration_scale = parameters.gravity.norm() / parameters.acc_norm;

        // init data
        reset();
    }

    void SmallPointLio::reset() {
        preprocess.reset();
        estimator.reset();
        is_init = false;
        active_map_scan_id = 0;
        pending_scan_map_points.clear();
        planar_reference_initialized = false;
        planar_tilt_rotation.setIdentity();
    }

    void SmallPointLio::on_point_cloud_callback(const std::vector<common::Point> &pointcloud) {
        preprocess.on_point_cloud_callback(pointcloud);
    }

    void SmallPointLio::on_imu_callback(const common::ImuMsg &imu_msg) {
        preprocess.on_imu_callback(imu_msg);
    }

    void SmallPointLio::handle_once() {
        // we need to init small point lio
        if (!is_init) {
            if ((!preprocess.point_deque.empty() || !preprocess.imu_deque.empty()) &&
                preprocess.point_deque.size() >= parameters.init_map_size &&
                (!parameters.fix_gravity_direction || preprocess.imu_deque.size() >= 200)) {
                // Fix gravity direction only from a recent stationary window.
                // The robot can bounce while being spawned in Gazebo, and a
                // gimbal-mounted IMU can later contain large angular/centripetal
                // motion. Neither is a valid gravity calibration interval.
                if (parameters.fix_gravity_direction) {
                    constexpr std::size_t init_imu_samples = 200;
                    const auto imu_begin = preprocess.imu_deque.end() - init_imu_samples;
                    Eigen::Matrix<state::value_type, 3, 1> avg_acc = Eigen::Matrix<state::value_type, 3, 1>::Zero();
                    for (auto it = imu_begin; it != preprocess.imu_deque.end(); ++it) {
                        const auto &imu_msg = *it;
                        avg_acc += imu_msg.linear_acceleration.cast<state::value_type>();
                    }
                    avg_acc /= static_cast<state::value_type>(init_imu_samples);

                    state::value_type max_gyro_norm = 0.0;
                    state::value_type acc_squared_error = 0.0;
                    for (auto it = imu_begin; it != preprocess.imu_deque.end(); ++it) {
                        const auto &imu_msg = *it;
                        max_gyro_norm = std::max(
                                max_gyro_norm,
                                imu_msg.angular_velocity.cast<state::value_type>().norm());
                        acc_squared_error +=
                                (imu_msg.linear_acceleration.cast<state::value_type>() - avg_acc).squaredNorm();
                    }
                    const auto acc_rms = std::sqrt(
                            acc_squared_error / static_cast<state::value_type>(init_imu_samples));
                    if (max_gyro_norm > 0.05 || acc_rms > 0.15) {
                        return;
                    }

                    Eigen::Matrix<state::value_type, 3, 1> body_up = avg_acc.normalized();
                    Eigen::Matrix<state::value_type, 3, 1> world_up = -parameters.gravity.cast<state::value_type>().normalized();
                    Eigen::Matrix<state::value_type, 3, 1> axis = body_up.cross(world_up);
                    state::value_type axis_norm = axis.norm();
                    if (axis_norm > 0) {
                        axis /= axis_norm;
                        state::value_type angle = std::acos(body_up.dot(world_up));
                        estimator.kf.x.rotation = Eigen::AngleAxis<state::value_type>(angle, axis).toRotationMatrix();
                    }
                    estimator.kf.x.gravity = parameters.gravity.cast<state::value_type>();
                } else {
                    estimator.kf.x.gravity = parameters.gravity.cast<state::value_type>();
                }
                estimator.kf.x.acceleration = -estimator.kf.x.rotation.transpose() * estimator.kf.x.gravity;
                apply_planar_constraint();

                // init map: transform points to world frame so they match runtime pointcloud_odom_frame
                for (const auto &point: preprocess.point_deque) {
                    Eigen::Matrix<state::value_type, 3, 1> point_imu_frame;
                    if (parameters.extrinsic_est_en) {
                        point_imu_frame = estimator.kf.x.offset_R_L_I * point.position.cast<state::value_type>() + estimator.kf.x.offset_T_L_I;
                    } else {
                        point_imu_frame = estimator.Lidar_R_wrt_IMU * point.position.cast<state::value_type>() + estimator.Lidar_T_wrt_IMU;
                    }
                    estimator.ivox->add_point((estimator.kf.x.rotation * point_imu_frame + estimator.kf.x.position).cast<float>());
                }
                // init time
                if (preprocess.point_deque.empty()) {
                    time_current = preprocess.imu_deque.back().timestamp;
                } else if (preprocess.imu_deque.empty()) {
                    time_current = preprocess.point_deque.back().timestamp;
                } else {
                    time_current = std::max(preprocess.point_deque.back().timestamp, preprocess.imu_deque.back().timestamp);
                }
                estimator.kf.init_timestamp(time_current);
                // clear data
                preprocess.point_deque.clear();
                preprocess.dense_point_deque.clear();
                preprocess.imu_deque.clear();
                is_init = true;
            }
            return;
        }

        // Dense points are only used for the registered-cloud visualization.
        // Never advance the primary filter for them: doing so changes the
        // state integration according to display-point density without a
        // matching covariance prediction or sensor update.
        bool is_publish_odometry = !preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty() &&
                                   preprocess.imu_deque.front().timestamp < preprocess.point_deque.back().timestamp;
        while (!preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty()) {
            const common::Point &point_lidar_frame = preprocess.point_deque.front();
            const common::Point &dense_point_lidar_frame = preprocess.dense_point_deque.front();
            const common::ImuMsg &imu_msg = preprocess.imu_deque.front();
            if (dense_point_lidar_frame.timestamp < point_lidar_frame.timestamp &&
                dense_point_lidar_frame.timestamp < imu_msg.timestamp) {
                Eigen::Matrix<state::value_type, 3, 1> dense_point_imu_frame;
                if (parameters.extrinsic_est_en) {
                    dense_point_imu_frame = estimator.kf.x.offset_R_L_I * dense_point_lidar_frame.position.cast<state::value_type>() + estimator.kf.x.offset_T_L_I;
                } else {
                    dense_point_imu_frame = estimator.Lidar_R_wrt_IMU * dense_point_lidar_frame.position.cast<state::value_type>() + estimator.Lidar_T_wrt_IMU;
                }
                pointcloud_odom_frame.emplace_back((estimator.kf.x.rotation * dense_point_imu_frame + estimator.kf.x.position).cast<float>());

                preprocess.dense_point_deque.pop_front();
            } else if (point_lidar_frame.timestamp < imu_msg.timestamp) {
                // point update
                if (point_lidar_frame.timestamp < time_current) {
                    preprocess.point_deque.pop_front();
                    continue;
                }
                time_current = point_lidar_frame.timestamp;

                if (parameters.defer_map_insertion_by_scan) {
                    begin_map_scan(point_lidar_frame.scan_id);
                }

                // predict
                estimator.kf.predict_state(time_current);

                // update
                estimator.point_lidar_frame = point_lidar_frame.position;
                estimator.kf.update_point();
                apply_planar_constraint();

                // publish odometry
                if (parameters.publish_odometry_without_downsample) {
                    publish_odometry(time_current);
                }

                // map incremental
                if (parameters.defer_map_insertion_by_scan) {
                    pending_scan_map_points.push_back(estimator.point_odom_frame);
                } else {
                    estimator.ivox->add_point(estimator.point_odom_frame);
                }

                preprocess.point_deque.pop_front();
            } else {
                // imu update
                if (imu_msg.timestamp < time_current) {
                    preprocess.imu_deque.pop_front();
                    continue;
                }
                time_current = imu_msg.timestamp;

                // predict
                estimator.kf.predict_state(time_current);
                estimator.kf.predict_cov(time_current, Q);

                // update
                estimator.angular_velocity = imu_msg.angular_velocity.cast<state::value_type>();
                estimator.linear_acceleration = imu_msg.linear_acceleration.cast<state::value_type>();
                estimator.kf.update_imu();
                apply_planar_constraint();

                preprocess.imu_deque.pop_front();
            }
        }

        if (is_publish_odometry) {
            if (!parameters.publish_odometry_without_downsample) {
                publish_odometry(time_current);
            }
            if (!pointcloud_odom_frame.empty()) {
                if (pointcloud_callback) {
                    pointcloud_callback(pointcloud_odom_frame);
                }
                pointcloud_odom_frame.clear();
            }
        }
    }

    void SmallPointLio::set_pointcloud_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> &pointcloud_callback) {
        this->pointcloud_callback = pointcloud_callback;
    }

    void SmallPointLio::set_odometry_callback(const std::function<void(const common::Odometry &odometry)> &odometry_callback) {
        this->odometry_callback = odometry_callback;
    }

    void SmallPointLio::begin_map_scan(std::uint64_t scan_id) {
        if (active_map_scan_id == 0) {
            active_map_scan_id = scan_id;
            return;
        }
        if (scan_id != active_map_scan_id) {
            flush_pending_map_points();
            active_map_scan_id = scan_id;
        }
    }

    void SmallPointLio::flush_pending_map_points() {
        for (const auto &point: pending_scan_map_points) {
            estimator.ivox->add_point(point);
        }
        pending_scan_map_points.clear();
    }

    void SmallPointLio::apply_planar_constraint() {
        if (!parameters.planar_constraint_en) {
            return;
        }

        auto &x = estimator.kf.x;
        const auto yaw = std::atan2(x.rotation(1, 0), x.rotation(0, 0));
        const auto cos_yaw = std::cos(yaw);
        const auto sin_yaw = std::sin(yaw);
        Eigen::Matrix<state::value_type, 3, 3> yaw_rotation;
        yaw_rotation << cos_yaw, -sin_yaw, 0.0,
                        sin_yaw,  cos_yaw, 0.0,
                        0.0,      0.0,     1.0;

        x.position.z() = static_cast<state::value_type>(parameters.planar_z);
        x.velocity.z() = 0.0;
        if (parameters.planar_preserve_initial_tilt) {
            // Legacy raw-sensor mode keeps the fixed mounting tilt.
            if (!planar_reference_initialized) {
                planar_tilt_rotation = yaw_rotation.transpose() * x.rotation;
                planar_reference_initialized = true;
            }
            x.rotation = yaw_rotation * planar_tilt_rotation;
        } else {
            // The compensated virtual body frame is level and planar.
            x.rotation = yaw_rotation;
            x.omg.x() = 0.0;
            x.omg.y() = 0.0;
        }

        auto &P = estimator.kf.P;
        constexpr state::value_type constrained_cov = 1e-6;
        const std::array<int, 4> constrained_indices = {
                state::position_index + 2,
                state::rotation_index + 0,
                state::rotation_index + 1,
                state::velocity_index + 2,
        };
        for (const auto index: constrained_indices) {
            P.row(index).setZero();
            P.col(index).setZero();
            P(index, index) = constrained_cov;
        }
        if (!parameters.planar_preserve_initial_tilt) {
            for (const auto index: {
                    state::omg_index + 0,
                    state::omg_index + 1}) {
                P.row(index).setZero();
                P.col(index).setZero();
                P(index, index) = constrained_cov;
            }
        }
    }

    void SmallPointLio::publish_odometry(double timestamp) {
        if (odometry_callback) {
            common::Odometry odometry;
            odometry.timestamp = timestamp;
            odometry.position = estimator.kf.x.position.cast<double>();
            odometry.velocity = estimator.kf.x.velocity.cast<double>();
            odometry.orientation = estimator.kf.x.rotation.cast<double>();
            odometry.angular_velocity = estimator.kf.x.omg.cast<double>();
            odometry_callback(odometry);
        }
    }

}// namespace small_point_lio
