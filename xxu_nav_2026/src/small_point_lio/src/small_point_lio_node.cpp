/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio_node.hpp"
#include "io/pcd_io.h"
#include "lidar_adapter/custom_mid360_driver.h"
#include "lidar_adapter/generic_pointcloud2.h"
#include "lidar_adapter/livox_custom_msg.h"
#include "lidar_adapter/livox_pointcloud2.h"
#include "lidar_adapter/unitree_lidar.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace small_point_lio {

namespace {

builtin_interfaces::msg::Time toRosTime(const double timestamp)
{
    const auto seconds = static_cast<int64_t>(std::floor(timestamp));
    auto nanoseconds = static_cast<int64_t>(std::llround((timestamp - static_cast<double>(seconds)) * 1e9));
    int64_t normalized_seconds = seconds;
    if (nanoseconds >= 1000000000LL) {
        ++normalized_seconds;
        nanoseconds -= 1000000000LL;
    }
    if (nanoseconds < 0) {
        --normalized_seconds;
        nanoseconds += 1000000000LL;
    }
    builtin_interfaces::msg::Time result;
    result.sec = static_cast<int32_t>(normalized_seconds);
    result.nanosec = static_cast<uint32_t>(nanoseconds);
    return result;
}

struct RigidTransform
{
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();

    [[nodiscard]] Eigen::Vector3d apply(const Eigen::Vector3d &point) const
    {
        return rotation * point + translation;
    }
};

RigidTransform fromRosTransform(const geometry_msgs::msg::TransformStamped &transform)
{
    RigidTransform result;
    result.rotation = Eigen::Quaterniond(
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z);
    const auto norm = result.rotation.norm();
    if (!std::isfinite(norm) || norm <= std::numeric_limits<double>::epsilon()) {
        throw std::runtime_error("TF contains an invalid quaternion");
    }
    result.rotation.normalize();
    result.translation << transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z;
    return result;
}

RigidTransform inverse(const RigidTransform &transform)
{
    RigidTransform result;
    result.rotation = transform.rotation.conjugate();
    result.translation = -(result.rotation * transform.translation);
    return result;
}

RigidTransform compose(const RigidTransform &first, const RigidTransform &second)
{
    RigidTransform result;
    result.rotation = first.rotation * second.rotation;
    result.translation = first.apply(second.translation);
    return result;
}

bool interpolatePose(
        const std::deque<common::Odometry> &history,
        const double timestamp,
        common::Odometry &result)
{
    constexpr double timestamp_epsilon = 1e-9;
    if (history.empty() || timestamp < history.front().timestamp - timestamp_epsilon ||
        timestamp > history.back().timestamp + timestamp_epsilon) {
        return false;
    }
    if (history.size() == 1 || timestamp <= history.front().timestamp + timestamp_epsilon) {
        result = history.front();
        result.timestamp = timestamp;
        return true;
    }
    if (timestamp >= history.back().timestamp - timestamp_epsilon) {
        result = history.back();
        result.timestamp = timestamp;
        return true;
    }

    const auto upper = std::upper_bound(
            history.begin(), history.end(), timestamp,
            [](const double value, const common::Odometry &sample) {
                return value < sample.timestamp;
            });
    if (upper == history.begin() || upper == history.end()) {
        return false;
    }
    const auto lower = std::prev(upper);
    const double interval = upper->timestamp - lower->timestamp;
    if (interval <= timestamp_epsilon) {
        result = *lower;
        result.timestamp = timestamp;
        return true;
    }
    const double alpha = (timestamp - lower->timestamp) / interval;
    result.timestamp = timestamp;
    result.position = lower->position + alpha * (upper->position - lower->position);
    result.orientation = lower->orientation.slerp(alpha, upper->orientation).normalized();
    result.velocity = lower->velocity + alpha * (upper->velocity - lower->velocity);
    result.angular_velocity = lower->angular_velocity + alpha * (upper->angular_velocity - lower->angular_velocity);
    return true;
}

}  // namespace

    SmallPointLioNode::SmallPointLioNode(const rclcpp::NodeOptions &options)
        : Node("small_point_lio", options) {
        std::string lidar_topic = declare_parameter<std::string>("lidar_topic");
        std::string imu_topic = declare_parameter<std::string>("imu_topic");
        std::string lidar_type = declare_parameter<std::string>("lidar_type");
        lidar_frame = declare_parameter<std::string>("lidar_frame");
        std::string odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
        const auto deskewed_cloud_topic = declare_parameter<std::string>("deskewed_cloud_topic", "/cloud_deskewed");
        std::string odom_frame = declare_parameter<std::string>("odom_frame", "odom");
        base_frame = declare_parameter<std::string>("base_frame", "base_link");
        tf_lookup_timeout = declare_parameter<double>("tf_lookup_timeout", 0.1);
        deskew_pending_timeout = declare_parameter<double>("deskew_pending_timeout", 0.25);
        deskew_pending_queue_size = static_cast<std::size_t>(std::max(
                1L, declare_parameter<long>("deskew_pending_queue_size", 3L)));
        bool save_pcd = declare_parameter<bool>("save_pcd");
        small_point_lio = std::make_unique<small_point_lio::SmallPointLio>(*this);
        get_parameter("deskew_pose_history_duration", deskew_pose_history_duration);
        if (!std::isfinite(tf_lookup_timeout) || tf_lookup_timeout < 0.0 ||
            !std::isfinite(deskew_pending_timeout) || deskew_pending_timeout < 0.0 ||
            !std::isfinite(deskew_pose_history_duration) || deskew_pose_history_duration <= 0.0) {
            throw std::invalid_argument("deskew timing parameters are invalid");
        }
        odometry_publisher = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1000);
        pointcloud_publisher = create_publisher<sensor_msgs::msg::PointCloud2>(deskewed_cloud_topic, 10);
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        deskew_retry_timer = create_wall_timer(
                std::chrono::milliseconds(10),
                std::bind(&SmallPointLioNode::process_pending_deskew_scans, this));
        if (save_pcd) {
            pointcloud_mapping = std::make_unique<util::PointcloudMapping>(0.02);
        }
        map_save_trigger = create_service<std_srvs::srv::Trigger>(
                "map_save",
                [this, save_pcd](const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res) {
                    if (!save_pcd) {
                        res->success = false;
                        res->message = "pcd save is disabled";
                        RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "pcd save is disabled");
                        return;
                    }
                    res->success = true;
                    RCLCPP_INFO(rclcpp::get_logger("small_point_lio"), "waiting for pcd saving ...");
                    auto pointcloud_to_save = std::make_shared<std::vector<Eigen::Vector3f>>();
                    *pointcloud_to_save = pointcloud_mapping->get_points();
                    std::thread([pointcloud_to_save]() {
                        io::pcd::write_pcd(ROOT_DIR + "/pcd/scan.pcd", *pointcloud_to_save);
                        RCLCPP_INFO(rclcpp::get_logger("small_point_lio"), "save pcd success");
                    }).detach();
                });
        small_point_lio->set_pose_history_callback(
                [this](const common::Odometry &odometry) { on_pose_history(odometry); });
        small_point_lio->set_deskew_scan_callback(
                [this](const std::vector<common::Point> &pointcloud) { on_dense_scan(pointcloud); });
        small_point_lio->set_odometry_callback([this, odom_frame](const common::Odometry &odometry) {
            const auto time_msg = toRosTime(odometry.timestamp);

            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = time_msg;
            transform_stamped.header.frame_id = odom_frame;
            transform_stamped.child_frame_id = base_frame;
            geometry_msgs::msg::TransformStamped base_link_to_lidar_frame_transform;
            try {
                base_link_to_lidar_frame_transform = tf_buffer->lookupTransform(
                        this->lidar_frame,
                        this->base_frame,
                        rclcpp::Time(time_msg),
                        rclcpp::Duration::from_seconds(this->tf_lookup_timeout));
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR_THROTTLE(
                        get_logger(),
                        *get_clock(),
                        1000,
                        "Failed to lookup transform from %s to %s: %s",
                        this->base_frame.c_str(),
                        this->lidar_frame.c_str(),
                        ex.what());
                return;
            }
            const auto lidar_from_base = fromRosTransform(base_link_to_lidar_frame_transform);
            const RigidTransform odom_from_lidar{
                    odometry.orientation,
                    odometry.position};
            const auto odom_from_base = compose(odom_from_lidar, lidar_from_base);

            auto &transform = transform_stamped.transform;
            transform.translation.x = odom_from_base.translation.x();
            transform.translation.y = odom_from_base.translation.y();
            transform.translation.z = odom_from_base.translation.z();
            transform.rotation.x = odom_from_base.rotation.x();
            transform.rotation.y = odom_from_base.rotation.y();
            transform.rotation.z = odom_from_base.rotation.z();
            transform.rotation.w = odom_from_base.rotation.w();

            nav_msgs::msg::Odometry odometry_msg;
            odometry_msg.header = transform_stamped.header;
            odometry_msg.child_frame_id = transform_stamped.child_frame_id;
            odometry_msg.pose.pose.position.x = transform_stamped.transform.translation.x;
            odometry_msg.pose.pose.position.y = transform_stamped.transform.translation.y;
            odometry_msg.pose.pose.position.z = transform_stamped.transform.translation.z;
            odometry_msg.pose.pose.orientation = transform_stamped.transform.rotation;

            // Odometry.twist is expressed in child_frame_id.  The LIO
            // callback velocity is at the lidar origin in odom coordinates;
            // first remove the lidar->base lever arm, then rotate to the
            // published base frame.  This remains correct for a spinning
            // gimbal and for a non-zero lidar/IMU extrinsic.
            const Eigen::Vector3d omega_odom =
                    odom_from_lidar.rotation * odometry.angular_velocity;
            const Eigen::Vector3d lidar_offset_odom =
                    odom_from_lidar.rotation * lidar_from_base.translation;
            const Eigen::Vector3d base_velocity_odom =
                    odometry.velocity - omega_odom.cross(lidar_offset_odom);
            const Eigen::Vector3d instantaneous_base_velocity =
                    odom_from_base.rotation.conjugate() * base_velocity_odom;
            const Eigen::Vector3d instantaneous_base_angular_velocity =
                    odom_from_base.rotation.conjugate() * omega_odom;

            // The pose above is the authoritative odom->base transform.  A
            // finite difference of that same transform avoids publishing a
            // velocity from a different state origin/frame when the lidar is
            // on a rotating or compensated gimbal.  Keep the transformed LIO
            // velocity as the first-sample/fallback value.
            Eigen::Vector3d base_velocity = instantaneous_base_velocity;
            Eigen::Vector3d base_angular_velocity = instantaneous_base_angular_velocity;
            const double dt = odometry.timestamp - last_published_pose_timestamp;
            if (has_last_published_pose && dt > 1.0e-6 && dt <= 1.0) {
                const Eigen::Vector3d delta_position =
                        odom_from_base.translation - last_published_position;
                base_velocity = odom_from_base.rotation.conjugate() * delta_position / dt;

                const Eigen::Quaterniond relative_rotation =
                        last_published_orientation.conjugate() * odom_from_base.rotation;
                const double delta_yaw = std::atan2(
                        2.0 * (relative_rotation.w() * relative_rotation.z() +
                                relative_rotation.x() * relative_rotation.y()),
                        1.0 - 2.0 * (relative_rotation.y() * relative_rotation.y() +
                                      relative_rotation.z() * relative_rotation.z()));
                base_angular_velocity = Eigen::Vector3d(0.0, 0.0, delta_yaw / dt);
            }
            last_published_pose_timestamp = odometry.timestamp;
            last_published_position = odom_from_base.translation;
            last_published_orientation = odom_from_base.rotation;
            has_last_published_pose = true;
            odometry_msg.twist.twist.linear.x = base_velocity.x();
            odometry_msg.twist.twist.linear.y = base_velocity.y();
            odometry_msg.twist.twist.linear.z = base_velocity.z();
            odometry_msg.twist.twist.angular.x = base_angular_velocity.x();
            odometry_msg.twist.twist.angular.y = base_angular_velocity.y();
            odometry_msg.twist.twist.angular.z = base_angular_velocity.z();

            // Keep the covariance conservative until a calibrated covariance
            // model is available; zeros make consumers treat the estimate as
            // exact and can destabilize localization filters.
            odometry_msg.twist.covariance[0] = 0.04;
            odometry_msg.twist.covariance[7] = 0.04;
            odometry_msg.twist.covariance[14] = 0.09;
            odometry_msg.twist.covariance[21] = 0.09;
            odometry_msg.twist.covariance[28] = 0.09;
            odometry_msg.twist.covariance[35] = 0.09;

            tf_broadcaster->sendTransform(transform_stamped);
            odometry_publisher->publish(odometry_msg);
        });
        if (lidar_type == "livox_custom_msg") {
#ifdef HAVE_LIVOX_DRIVER
            lidar_adapter = std::make_unique<LivoxCustomMsgAdapter>();
#else
            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "livox_custom_msg requested but not available!");
            rclcpp::shutdown();
            return;
#endif
        } else if (lidar_type == "livox_pointcloud2") {
            lidar_adapter = std::make_unique<LivoxPointCloud2Adapter>();
        } else if (lidar_type == "custom_mid360_driver") {
            lidar_adapter = std::make_unique<CustomMid360DriverAdapter>();
        } else if (lidar_type == "generic_pointcloud2") {
            lidar_adapter = std::make_unique<GenericPointCloud2Adapter>();
        } else if (lidar_type == "unilidar") {
            lidar_adapter = std::make_unique<UnilidarAdapter>();
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "unknown lidar type: %s", lidar_type.c_str());
            rclcpp::shutdown();
            return;
        }
        lidar_adapter->setup_subscription(this, lidar_topic, [this](const std::vector<common::Point> &pointcloud) {
            small_point_lio->on_point_cloud_callback(pointcloud);
            small_point_lio->handle_once();
        });
        imu_subsciber = create_subscription<sensor_msgs::msg::Imu>(
                imu_topic,
                rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Imu &msg) {
                    common::ImuMsg imu_msg;
                    imu_msg.angular_velocity = Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
                    imu_msg.linear_acceleration = Eigen::Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
                    imu_msg.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
                    small_point_lio->on_imu_callback(imu_msg);
                    small_point_lio->handle_once();
                });
    }

    void SmallPointLioNode::on_pose_history(const common::Odometry &odometry) {
        if (!std::isfinite(odometry.timestamp) || !odometry.position.allFinite() ||
            !odometry.orientation.coeffs().allFinite() ||
            odometry.orientation.norm() <= std::numeric_limits<double>::epsilon()) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Ignoring invalid LIO pose history sample");
            return;
        }

        auto sample = odometry;
        sample.orientation.normalize();
        constexpr double timestamp_epsilon = 1e-9;
        std::lock_guard<std::mutex> lock(deskew_mutex);
        if (!pose_history.empty() &&
            sample.timestamp < pose_history.back().timestamp - timestamp_epsilon) {
            return;
        }
        if (!pose_history.empty() &&
            std::abs(sample.timestamp - pose_history.back().timestamp) <= timestamp_epsilon) {
            pose_history.back() = sample;
        } else {
            pose_history.push_back(sample);
        }
        while (pose_history.size() > 2 &&
               pose_history.back().timestamp - pose_history.front().timestamp >
               deskew_pose_history_duration) {
            pose_history.pop_front();
        }
    }

    void SmallPointLioNode::on_dense_scan(const std::vector<common::Point> &pointcloud) {
        if (pointcloud.empty()) {
            return;
        }
        double start_timestamp = std::numeric_limits<double>::infinity();
        double end_timestamp = -std::numeric_limits<double>::infinity();
        for (const auto &point : pointcloud) {
            if (std::isfinite(point.timestamp)) {
                start_timestamp = std::min(start_timestamp, point.timestamp);
                end_timestamp = std::max(end_timestamp, point.timestamp);
            }
        }
        if (!std::isfinite(end_timestamp)) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Dropping dense scan without finite point timestamps");
            return;
        }

        PendingDeskewScan pending;
        pending.points = pointcloud;
        pending.start_timestamp = start_timestamp;
        pending.end_timestamp = end_timestamp;
        pending.queued_at = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(deskew_mutex);
            if (pending_deskew_scans.size() >= deskew_pending_queue_size) {
                pending_deskew_scans.pop_front();
                RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Dropping oldest pending dense scan because the deskew queue is full");
            }
            pending_deskew_scans.push_back(std::move(pending));
        }
        process_pending_deskew_scans();
    }

    void SmallPointLioNode::process_pending_deskew_scans() {
        std::lock_guard<std::mutex> process_lock(deskew_process_mutex);
        while (rclcpp::ok()) {
            PendingDeskewScan pending;
            std::deque<common::Odometry> history;
            {
                std::lock_guard<std::mutex> lock(deskew_mutex);
                if (pending_deskew_scans.empty()) {
                    return;
                }
                pending = pending_deskew_scans.front();
                history = pose_history;
            }

            const auto age = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - pending.queued_at).count();
            common::Odometry pose_at_end;
            bool pose_data_waiting = history.empty() ||
                pending.end_timestamp > history.back().timestamp + 1e-9;
            if (!pose_data_waiting && !interpolatePose(history, pending.end_timestamp, pose_at_end)) {
                pose_data_waiting = true;
            }
            if (!pose_data_waiting && pending.start_timestamp < history.front().timestamp - 1e-9) {
                RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Dropping dense scan at %.6f: pose history has already expired",
                        pending.end_timestamp);
                std::lock_guard<std::mutex> lock(deskew_mutex);
                if (!pending_deskew_scans.empty()) {
                    pending_deskew_scans.pop_front();
                }
                continue;
            }
            if (pose_data_waiting) {
                if (age <= deskew_pending_timeout) {
                    return;
                }
                RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Dropping dense scan at %.6f after %.3f s without complete LIO pose history",
                        pending.end_timestamp, age);
                std::lock_guard<std::mutex> lock(deskew_mutex);
                if (!pending_deskew_scans.empty()) {
                    pending_deskew_scans.pop_front();
                }
                continue;
            }

            geometry_msgs::msg::TransformStamped lidar_from_base_msg;
            try {
                // This is an exact-time lookup. A newer TF is never used as a
                // substitute for the transform at the scan end.
                lidar_from_base_msg = tf_buffer->lookupTransform(
                        lidar_frame,
                        base_frame,
                        rclcpp::Time(toRosTime(pending.end_timestamp)),
                        rclcpp::Duration::from_seconds(tf_lookup_timeout));
            } catch (const tf2::TransformException &ex) {
                if (age <= deskew_pending_timeout) {
                    return;
                }
                RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Dropping dense scan at %.6f after %.3f s without lidar/base TF: %s",
                        pending.end_timestamp, age, ex.what());
                std::lock_guard<std::mutex> lock(deskew_mutex);
                if (!pending_deskew_scans.empty()) {
                    pending_deskew_scans.pop_front();
                }
                continue;
            } catch (const std::exception &ex) {
                RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Dropping malformed TF for dense scan at %.6f: %s",
                        pending.end_timestamp, ex.what());
                std::lock_guard<std::mutex> lock(deskew_mutex);
                if (!pending_deskew_scans.empty()) {
                    pending_deskew_scans.pop_front();
                }
                continue;
            }

            try {
                const auto lidar_from_base = fromRosTransform(lidar_from_base_msg);
                const auto odom_from_lidar_end = RigidTransform{
                        pose_at_end.orientation,
                        pose_at_end.position};
                const auto odom_from_base_end = compose(odom_from_lidar_end, lidar_from_base);
                const auto base_from_odom_end = inverse(odom_from_base_end);

                std::vector<Eigen::Vector3f> odom_points;
                std::vector<Eigen::Vector3f> base_points;
                odom_points.reserve(pending.points.size());
                base_points.reserve(pending.points.size());
                for (const auto &point : pending.points) {
                    if (!std::isfinite(point.timestamp) || !point.position.allFinite()) {
                        continue;
                    }
                    common::Odometry pose_at_point;
                    if (!interpolatePose(history, point.timestamp, pose_at_point)) {
                        throw std::runtime_error("LIO pose history does not bracket every point");
                    }
                    const RigidTransform odom_from_lidar{
                            pose_at_point.orientation,
                            pose_at_point.position};
                    const auto point_in_odom = odom_from_lidar.apply(point.position.cast<double>());
                    const auto point_in_base = base_from_odom_end.apply(point_in_odom);
                    if (!point_in_odom.allFinite() || !point_in_base.allFinite()) {
                        continue;
                    }
                    odom_points.push_back(point_in_odom.cast<float>());
                    base_points.push_back(point_in_base.cast<float>());
                }

                if (base_points.empty()) {
                    RCLCPP_WARN_THROTTLE(
                            get_logger(), *get_clock(), 2000,
                            "Dropping empty dense scan at %.6f after finite-point filtering",
                            pending.end_timestamp);
                } else {
                    if (pointcloud_mapping) {
                        pointcloud_mapping->add_pointcloud(odom_points);
                    }
                    sensor_msgs::msg::PointCloud2 output;
                    output.header.stamp = toRosTime(pending.end_timestamp);
                    output.header.frame_id = base_frame;
                    output.height = 1;
                    output.width = static_cast<uint32_t>(base_points.size());
                    output.fields.resize(4);
                    output.fields[0].name = "x";
                    output.fields[0].offset = 0;
                    output.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
                    output.fields[0].count = 1;
                    output.fields[1].name = "y";
                    output.fields[1].offset = 4;
                    output.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
                    output.fields[1].count = 1;
                    output.fields[2].name = "z";
                    output.fields[2].offset = 8;
                    output.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
                    output.fields[2].count = 1;
                    output.fields[3].name = "intensity";
                    output.fields[3].offset = 12;
                    output.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
                    output.fields[3].count = 1;
                    output.is_bigendian = false;
                    output.point_step = 16;
                    output.row_step = output.width * output.point_step;
                    output.data.resize(output.row_step);
                    for (size_t index = 0; index < base_points.size(); ++index) {
                        const size_t offset = index * output.point_step;
                        const float values[4] = {
                                base_points[index].x(), base_points[index].y(),
                                base_points[index].z(), 0.0F};
                        std::memcpy(output.data.data() + offset, values, sizeof(values));
                    }
                    output.is_dense = true;
                    pointcloud_publisher->publish(output);
                }
            } catch (const std::exception &ex) {
                RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Dropping dense scan at %.6f: %s", pending.end_timestamp, ex.what());
            }

            std::lock_guard<std::mutex> lock(deskew_mutex);
            if (!pending_deskew_scans.empty()) {
                pending_deskew_scans.pop_front();
            }
        }
    }

}// namespace small_point_lio

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(small_point_lio::SmallPointLioNode)
