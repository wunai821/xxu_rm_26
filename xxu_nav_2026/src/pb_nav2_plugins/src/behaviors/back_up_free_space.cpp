// Copyright 2024 Polaris Xia
// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb_nav2_plugins/behaviors/back_up_free_space.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/utils.h"
#include "nav2_util/node_utils.hpp"

namespace pb_nav2_behaviors
{

void BackUpFreeSpace::onConfigure()
{
  nav2_behaviors::DriveOnHeading<BackUpAction>::onConfigure();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(node, "global_frame", rclcpp::ParameterValue("map"));
  nav2_util::declare_parameter_if_not_declared(node, "max_radius", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "service_name", rclcpp::ParameterValue("local_costmap/get_costmap"));
  nav2_util::declare_parameter_if_not_declared(node, "visualize", rclcpp::ParameterValue(false));

  node->get_parameter("global_frame", global_frame_);
  node->get_parameter("max_radius", max_radius_);
  node->get_parameter("service_name", service_name_);
  node->get_parameter("visualize", visualize_);

  costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);

  if (visualize_) {
    marker_pub_ = node->template create_publisher<visualization_msgs::msg::MarkerArray>(
      "back_up_free_space_markers", 1);
    marker_pub_->on_activate();
  }
}

void BackUpFreeSpace::onCleanup()
{
  costmap_client_.reset();
  marker_pub_.reset();
}

nav2_behaviors::ResultStatus BackUpFreeSpace::onRun(
  const std::shared_ptr<const BackUpActionGoal> command)
{
  if (command->target.y != 0.0 || command->target.z != 0.0) {
    RCLCPP_INFO(logger_, "BackUpFreeSpace ignores Y/Z targets; use target.x as travel distance.");
    return {nav2_behaviors::Status::FAILED, BackUpActionResult::INVALID_INPUT};
  }

  const double command_speed = std::fabs(command->speed);
  if (command_speed <= 0.0) {
    RCLCPP_ERROR(logger_, "BackUpFreeSpace requires a non-zero speed.");
    return {nav2_behaviors::Status::FAILED, BackUpActionResult::INVALID_INPUT};
  }

  while (!costmap_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for %s.", service_name_.c_str());
      return {nav2_behaviors::Status::FAILED, BackUpActionResult::UNKNOWN};
    }
    RCLCPP_WARN(logger_, "Waiting for costmap service %s...", service_name_.c_str());
  }

  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto result = costmap_client_->async_send_request(request);
  if (result.wait_for(std::chrono::seconds(2)) == std::future_status::timeout) {
    RCLCPP_ERROR(logger_, "Timed out while requesting costmap from %s.", service_name_.c_str());
    return {nav2_behaviors::Status::FAILED, BackUpActionResult::UNKNOWN};
  }

  const auto costmap = result.get()->map;
  costmap_frame_ = costmap.header.frame_id.empty() ? global_frame_ : costmap.header.frame_id;

  if (!nav2_util::getCurrentPose(
      initial_pose_, *tf_, costmap_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available in %s.", costmap_frame_.c_str());
    return {nav2_behaviors::Status::FAILED, BackUpActionResult::TF_ERROR};
  }

  geometry_msgs::msg::Pose2D pose;
  pose.x = initial_pose_.pose.position.x;
  pose.y = initial_pose_.pose.position.y;
  pose.theta = tf2::getYaw(initial_pose_.pose.orientation);

  float best_angle = 0.0f;
  if (!findBestDirection(
      costmap, pose, -M_PI, M_PI, static_cast<float>(max_radius_), M_PI / 32.0f, best_angle))
  {
    RCLCPP_WARN(logger_, "No free backup direction found in %s.", service_name_.c_str());
    return {nav2_behaviors::Status::FAILED, BackUpActionResult::COLLISION_AHEAD};
  }

  const double relative_angle = static_cast<double>(best_angle) - pose.theta;
  twist_x_ = std::cos(relative_angle) * command_speed;
  twist_y_ = -std::sin(relative_angle) * command_speed;
  command_x_ = std::fabs(command->target.x);
  command_time_allowance_ = command->time_allowance;
  end_time_ = clock_->now() + command_time_allowance_;

  RCLCPP_WARN(
    logger_,
    "Backing up %.3f m toward free space in %s at %.3f rad; cmd=(%.3f, %.3f)",
    command_x_, costmap_frame_.c_str(), best_angle, twist_x_, twist_y_);

  return {nav2_behaviors::Status::SUCCEEDED, BackUpActionResult::NONE};
}

nav2_behaviors::ResultStatus BackUpFreeSpace::onCycleUpdate()
{
  rclcpp::Duration time_remaining = end_time_ - clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(logger_, "Exceeded time allowance before reaching the BackUpFreeSpace goal.");
    return {nav2_behaviors::Status::FAILED, BackUpActionResult::TIMEOUT};
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, costmap_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available in %s.", costmap_frame_.c_str());
    return {nav2_behaviors::Status::FAILED, BackUpActionResult::TF_ERROR};
  }

  const double diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
  const double diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
  const double distance = std::hypot(diff_x, diff_y);

  feedback_->distance_traveled = static_cast<float>(distance);
  action_server_->publish_feedback(feedback_);

  if (distance >= std::fabs(command_x_)) {
    stopRobot();
    return {nav2_behaviors::Status::SUCCEEDED, BackUpActionResult::NONE};
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.stamp = clock_->now();
  cmd_vel->header.frame_id = robot_base_frame_;
  cmd_vel->twist.linear.x = twist_x_;
  cmd_vel->twist.linear.y = twist_y_;

  geometry_msgs::msg::Pose2D pose;
  pose.x = current_pose.pose.position.x;
  pose.y = current_pose.pose.position.y;
  pose.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!isCollisionFree(distance, cmd_vel->twist, pose)) {
    stopRobot();
    RCLCPP_WARN(logger_, "Collision ahead - exiting BackUpFreeSpace.");
    return {nav2_behaviors::Status::FAILED, BackUpActionResult::COLLISION_AHEAD};
  }

  vel_pub_->publish(std::move(cmd_vel));

  return {nav2_behaviors::Status::RUNNING, BackUpActionResult::NONE};
}

bool BackUpFreeSpace::findBestDirection(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float start_angle,
  float end_angle, float radius, float angle_increment, float & best_angle)
{
  if (costmap.metadata.resolution <= 0.0f || costmap.data.empty() || angle_increment <= 0.0f) {
    return false;
  }

  float current_safe_start = std::numeric_limits<float>::quiet_NaN();
  float best_safe_start = 0.0f;
  float best_safe_end = 0.0f;
  float best_width = -1.0f;

  auto close_safe_segment = [&](float segment_end) {
    if (std::isnan(current_safe_start)) {
      return;
    }
    const float width = segment_end - current_safe_start;
    if (width > best_width) {
      best_width = width;
      best_safe_start = current_safe_start;
      best_safe_end = segment_end;
    }
    current_safe_start = std::numeric_limits<float>::quiet_NaN();
  };

  for (float angle = start_angle; angle <= end_angle + 1e-4f; angle += angle_increment) {
    const bool safe = isDirectionSafe(costmap, pose, angle, radius);
    if (safe && std::isnan(current_safe_start)) {
      current_safe_start = angle;
    } else if (!safe) {
      close_safe_segment(angle - angle_increment);
    }
  }
  close_safe_segment(end_angle);

  if (best_width < 0.0f) {
    return false;
  }

  best_angle = (best_safe_start + best_safe_end) * 0.5f;

  if (visualize_) {
    visualize(pose, radius, best_safe_start, best_safe_end);
  }

  return true;
}

bool BackUpFreeSpace::isDirectionSafe(
  const nav2_msgs::msg::Costmap & costmap, const geometry_msgs::msg::Pose2D & pose,
  float angle, float radius)
{
  const float resolution = costmap.metadata.resolution;
  const float origin_x = costmap.metadata.origin.position.x;
  const float origin_y = costmap.metadata.origin.position.y;
  const int size_x = static_cast<int>(costmap.metadata.size_x);
  const int size_y = static_cast<int>(costmap.metadata.size_y);

  const float map_min_x = origin_x;
  const float map_max_x = origin_x + size_x * resolution;
  const float map_min_y = origin_y;
  const float map_max_y = origin_y + size_y * resolution;

  for (float r = 0.0f; r <= radius; r += resolution) {
    const float x = pose.x + r * std::cos(angle);
    const float y = pose.y + r * std::sin(angle);

    if (x < map_min_x || x >= map_max_x || y < map_min_y || y >= map_max_y) {
      return false;
    }

    const int i = static_cast<int>((x - origin_x) / resolution);
    const int j = static_cast<int>((y - origin_y) / resolution);
    const auto idx = static_cast<size_t>(i + j * size_x);
    if (i < 0 || i >= size_x || j < 0 || j >= size_y || idx >= costmap.data.size()) {
      return false;
    }
    if (costmap.data[idx] >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }
  }

  return true;
}

bool BackUpFreeSpace::isCollisionFree(
  const double & distance, const geometry_msgs::msg::Twist & cmd_vel,
  geometry_msgs::msg::Pose2D & pose2d)
{
  int cycle_count = 0;
  const double remaining_distance = std::fabs(command_x_) - distance;
  const int max_cycle_count = static_cast<int>(cycle_frequency_ * simulate_ahead_time_);
  const geometry_msgs::msg::Pose2D initial_pose = pose2d;
  bool fetch_data = true;

  while (cycle_count < max_cycle_count) {
    const double dt = cycle_count / cycle_frequency_;
    const double dx = (
      cmd_vel.linear.x * std::cos(initial_pose.theta) -
      cmd_vel.linear.y * std::sin(initial_pose.theta)) * dt;
    const double dy = (
      cmd_vel.linear.x * std::sin(initial_pose.theta) +
      cmd_vel.linear.y * std::cos(initial_pose.theta)) * dt;
    pose2d.x = initial_pose.x + dx;
    pose2d.y = initial_pose.y + dy;
    cycle_count++;

    if (remaining_distance - std::hypot(dx, dy) <= 0.0) {
      break;
    }

    if (!local_collision_checker_->isCollisionFree(pose2d, fetch_data)) {
      return false;
    }
    fetch_data = false;
  }

  return true;
}

std::vector<geometry_msgs::msg::Point> BackUpFreeSpace::gatherFreePoints(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float radius)
{
  std::vector<geometry_msgs::msg::Point> results;
  for (unsigned int i = 0; i < costmap.metadata.size_x; i++) {
    for (unsigned int j = 0; j < costmap.metadata.size_y; j++) {
      auto idx = i + j * costmap.metadata.size_x;
      auto x = i * costmap.metadata.resolution + costmap.metadata.origin.position.x;
      auto y = j * costmap.metadata.resolution + costmap.metadata.origin.position.y;
      if (std::hypot(x - pose.x, y - pose.y) <= radius && costmap.data[idx] == 0) {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        results.push_back(p);
      }
    }
  }
  return results;
}

void BackUpFreeSpace::visualize(
  geometry_msgs::msg::Pose2D pose, float radius, float first_safe_angle, float last_unsafe_angle)
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker sector_marker;
  sector_marker.header.frame_id = costmap_frame_.empty() ? global_frame_ : costmap_frame_;
  sector_marker.header.stamp = clock_->now();
  sector_marker.ns = "direction";
  sector_marker.id = 0;
  sector_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  sector_marker.action = visualization_msgs::msg::Marker::ADD;
  sector_marker.scale.x = 1.0;
  sector_marker.scale.y = 1.0;
  sector_marker.scale.z = 1.0;
  sector_marker.color.r = 0.0f;
  sector_marker.color.g = 1.0f;
  sector_marker.color.b = 0.0f;
  sector_marker.color.a = 0.2f;

  const float angle_step = 0.05f;
  for (float angle = first_safe_angle; angle <= last_unsafe_angle; angle += angle_step) {
    const float next_angle = std::min(angle + angle_step, last_unsafe_angle);

    geometry_msgs::msg::Point origin;
    origin.x = pose.x;
    origin.y = pose.y;
    origin.z = 0.0;

    geometry_msgs::msg::Point p1;
    p1.x = pose.x + radius * std::cos(angle);
    p1.y = pose.y + radius * std::sin(angle);
    p1.z = 0.0;

    geometry_msgs::msg::Point p2;
    p2.x = pose.x + radius * std::cos(next_angle);
    p2.y = pose.y + radius * std::sin(next_angle);
    p2.z = 0.0;

    sector_marker.points.push_back(origin);
    sector_marker.points.push_back(p1);
    sector_marker.points.push_back(p2);
  }
  markers.markers.push_back(sector_marker);

  auto create_arrow = [&](float angle, int id, float r, float g, float b) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = costmap_frame_.empty() ? global_frame_ : costmap_frame_;
    arrow.header.stamp = clock_->now();
    arrow.ns = "direction";
    arrow.id = id;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.05;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 1.0;

    geometry_msgs::msg::Point start;
    start.x = pose.x;
    start.y = pose.y;
    start.z = 0.0;

    geometry_msgs::msg::Point end;
    end.x = start.x + radius * std::cos(angle);
    end.y = start.y + radius * std::sin(angle);
    end.z = 0.0;

    arrow.points.push_back(start);
    arrow.points.push_back(end);
    return arrow;
  };

  markers.markers.push_back(create_arrow(first_safe_angle, 1, 0.0f, 0.0f, 1.0f));
  markers.markers.push_back(create_arrow(last_unsafe_angle, 2, 0.0f, 0.0f, 1.0f));

  const float best_angle = (first_safe_angle + last_unsafe_angle) / 2.0f;
  markers.markers.push_back(create_arrow(best_angle, 3, 0.0f, 1.0f, 0.0f));

  marker_pub_->publish(markers);
}

}  // namespace pb_nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pb_nav2_behaviors::BackUpFreeSpace, nav2_core::Behavior)
