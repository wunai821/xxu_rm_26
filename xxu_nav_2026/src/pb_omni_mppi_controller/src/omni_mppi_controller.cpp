// Copyright 2026 Lihan Chen
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

#include "pb_omni_mppi_controller/omni_mppi_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace pb_omni_mppi_controller {

namespace {

double sqr(double value) { return value * value; }

// clang-format off
}  // namespace
// clang-format on

void OmniMppiController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock lifecycle node");
  }

  node_ = parent;
  plugin_name_ = std::move(name);
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();

  declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance",
                                    rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".time_steps",
                                    rclcpp::ParameterValue(15));
  declare_parameter_if_not_declared(node, plugin_name_ + ".batch_size",
                                    rclcpp::ParameterValue(256));
  declare_parameter_if_not_declared(node, plugin_name_ + ".model_dt",
                                    rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".temperature",
                                    rclcpp::ParameterValue(0.35));
  declare_parameter_if_not_declared(node, plugin_name_ + ".noise_std_x",
                                    rclcpp::ParameterValue(0.35));
  declare_parameter_if_not_declared(node, plugin_name_ + ".noise_std_y",
                                    rclcpp::ParameterValue(0.35));
  declare_parameter_if_not_declared(node, plugin_name_ + ".noise_std_theta",
                                    rclcpp::ParameterValue(0.25));

  declare_parameter_if_not_declared(node, plugin_name_ + ".vx_min",
                                    rclcpp::ParameterValue(-1.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".vx_max",
                                    rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".vy_min",
                                    rclcpp::ParameterValue(-1.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".vy_max",
                                    rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".wz_min",
                                    rclcpp::ParameterValue(-0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".wz_max",
                                    rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_accel_x",
                                    rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_accel_y",
                                    rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_accel_theta",
                                    rclcpp::ParameterValue(1.5));

  declare_parameter_if_not_declared(node, plugin_name_ + ".path_weight",
                                    rclcpp::ParameterValue(8.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".heading_weight",
                                    rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node,
                                    plugin_name_ + ".terminal_path_weight",
                                    rclcpp::ParameterValue(16.0));
  declare_parameter_if_not_declared(node,
                                    plugin_name_ + ".terminal_heading_weight",
                                    rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".control_weight",
                                    rclcpp::ParameterValue(0.04));
  declare_parameter_if_not_declared(node, plugin_name_ + ".smoothness_weight",
                                    rclcpp::ParameterValue(0.12));
  declare_parameter_if_not_declared(node, plugin_name_ + ".collision_weight",
                                    rclcpp::ParameterValue(120.0));
  declare_parameter_if_not_declared(
      node, plugin_name_ + ".collision_cost_threshold",
      rclcpp::ParameterValue(
          static_cast<int>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)));
  declare_parameter_if_not_declared(
      node, plugin_name_ + ".consider_unknown_as_collision",
      rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
      node, plugin_name_ + ".max_robot_pose_search_dist",
      rclcpp::ParameterValue(2.0));

  auto get_double = [node, this](const std::string &suffix, double &target) {
    node->get_parameter(plugin_name_ + "." + suffix, target);
  };
  auto get_int = [node, this](const std::string &suffix, int &target) {
    node->get_parameter(plugin_name_ + "." + suffix, target);
  };

  get_double("transform_tolerance", transform_tolerance_);
  get_int("time_steps", time_steps_);
  get_int("batch_size", batch_size_);
  get_double("model_dt", model_dt_);
  get_double("temperature", temperature_);
  get_double("noise_std_x", noise_std_x_);
  get_double("noise_std_y", noise_std_y_);
  get_double("noise_std_theta", noise_std_theta_);
  get_double("vx_min", vx_min_);
  get_double("vx_max", vx_max_);
  get_double("vy_min", vy_min_);
  get_double("vy_max", vy_max_);
  get_double("wz_min", wz_min_);
  get_double("wz_max", wz_max_);
  get_double("max_accel_x", max_accel_x_);
  get_double("max_accel_y", max_accel_y_);
  get_double("max_accel_theta", max_accel_theta_);
  get_double("path_weight", path_weight_);
  get_double("heading_weight", heading_weight_);
  get_double("terminal_path_weight", terminal_path_weight_);
  get_double("terminal_heading_weight", terminal_heading_weight_);
  get_double("control_weight", control_weight_);
  get_double("smoothness_weight", smoothness_weight_);
  get_double("collision_weight", collision_weight_);
  get_double("max_robot_pose_search_dist", max_robot_pose_search_dist_);
  int collision_threshold = static_cast<int>(collision_cost_threshold_);
  node->get_parameter(plugin_name_ + ".collision_cost_threshold",
                      collision_threshold);
  collision_cost_threshold_ =
      static_cast<unsigned char>(std::clamp(collision_threshold, 1, 255));
  node->get_parameter(plugin_name_ + ".consider_unknown_as_collision",
                      consider_unknown_as_collision_);

  if (time_steps_ < 2 || batch_size_ < 2 || model_dt_ <= 0.0 ||
      temperature_ <= 0.0 || vx_min_ > vx_max_ || vy_min_ > vy_max_ ||
      wz_min_ > wz_max_ || max_accel_x_ <= 0.0 || max_accel_y_ <= 0.0 ||
      max_accel_theta_ <= 0.0) {
    throw nav2_core::ControllerException("Invalid MPPI parameter relation");
  }

  nominal_controls_.assign(static_cast<std::size_t>(time_steps_), Control{});
  nominal_initialized_ = false;
  configured_ = true;
}

void OmniMppiController::cleanup() {
  std::lock_guard<std::mutex> lock(mutex_);
  parameter_callback_handle_.reset();
  nominal_controls_.clear();
  global_plan_.poses.clear();
  costmap_ = nullptr;
  costmap_ros_.reset();
  tf_.reset();
  clock_.reset();
  configured_ = false;
  nominal_initialized_ = false;
}

void OmniMppiController::activate() {
  auto node = node_.lock();
  if (node) {
    parameter_callback_handle_ = node->add_on_set_parameters_callback(
        std::bind(&OmniMppiController::dynamicParametersCallback, this,
                  std::placeholders::_1));
  }
}

void OmniMppiController::deactivate() {
  std::lock_guard<std::mutex> lock(mutex_);
  parameter_callback_handle_.reset();
}

void OmniMppiController::setPlan(const nav_msgs::msg::Path &path) {
  std::lock_guard<std::mutex> lock(mutex_);
  global_plan_ = path;
  nominal_controls_.assign(static_cast<std::size_t>(time_steps_), Control{});
  nominal_initialized_ = false;
}

void OmniMppiController::setSpeedLimit(const double &speed_limit,
                                       const bool &percentage) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(speed_limit) || speed_limit < 0.0) {
    RCLCPP_WARN(logger_, "Ignoring invalid MPPI speed limit: %.3f",
                speed_limit);
    return;
  }
  speed_limit_factor_ = percentage ? speed_limit / 100.0 : speed_limit;
  if (percentage) {
    speed_limit_factor_ = std::clamp(speed_limit_factor_, 0.0, 1.0);
  }
}

geometry_msgs::msg::TwistStamped OmniMppiController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker * /*goal_checker*/) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_ || !costmap_ros_ || !costmap_) {
    throw nav2_core::ControllerException("MPPI controller is not configured");
  }

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(
      *costmap_->getMutex());
  const auto transformed_plan = transformGlobalPlan(pose);
  if (transformed_plan.poses.empty()) {
    throw nav2_core::InvalidPath("MPPI received an empty local plan");
  }

  std::vector<double> cumulative_distances(transformed_plan.poses.size(), 0.0);
  for (std::size_t i = 1; i < transformed_plan.poses.size(); ++i) {
    cumulative_distances[i] = cumulative_distances[i - 1] +
                              euclidean_distance(transformed_plan.poses[i - 1],
                                                 transformed_plan.poses[i]);
  }

  geometry_msgs::msg::PoseStamped base_pose;
  base_pose.header.frame_id = costmap_ros_->getBaseFrameID();
  base_pose.header.stamp = pose.header.stamp;
  base_pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::PoseStamped robot_pose_in_costmap;
  if (!transformPose(costmap_ros_->getGlobalFrameID(), base_pose,
                     robot_pose_in_costmap)) {
    throw nav2_core::ControllerTFError(
        "Unable to transform base pose into costmap frame");
  }

  if (!nominal_initialized_ ||
      nominal_controls_.size() != static_cast<std::size_t>(time_steps_)) {
    nominal_controls_.assign(static_cast<std::size_t>(time_steps_), Control{});
    const Control current{std::clamp(velocity.linear.x, vx_min_, vx_max_),
                          std::clamp(velocity.linear.y, vy_min_, vy_max_),
                          std::clamp(velocity.angular.z, wz_min_, wz_max_)};
    for (auto &control : nominal_controls_) {
      control = current;
    }
    nominal_initialized_ = true;
  }

  std::normal_distribution<double> noise_x(0.0, noise_std_x_);
  std::normal_distribution<double> noise_y(0.0, noise_std_y_);
  std::normal_distribution<double> noise_theta(0.0, noise_std_theta_);
  std::vector<std::vector<Control>> noises(
      static_cast<std::size_t>(batch_size_),
      std::vector<Control>(static_cast<std::size_t>(time_steps_), Control{}));
  std::vector<double> costs(static_cast<std::size_t>(batch_size_), 0.0);

  double min_cost = std::numeric_limits<double>::max();
  bool any_feasible = false;
  for (int sample = 0; sample < batch_size_; ++sample) {
    std::vector<Control> controls = nominal_controls_;
    Control previous{std::clamp(velocity.linear.x, vx_min_, vx_max_),
                     std::clamp(velocity.linear.y, vy_min_, vy_max_),
                     std::clamp(velocity.angular.z, wz_min_, wz_max_)};
    for (int step = 0; step < time_steps_; ++step) {
      if (sample != 0) {
        noises[static_cast<std::size_t>(sample)]
              [static_cast<std::size_t>(step)] =
                  Control{noise_x(random_engine_), noise_y(random_engine_),
                          noise_theta(random_engine_)};
        controls[static_cast<std::size_t>(step)].vx +=
            noises[static_cast<std::size_t>(sample)]
                  [static_cast<std::size_t>(step)]
                      .vx;
        controls[static_cast<std::size_t>(step)].vy +=
            noises[static_cast<std::size_t>(sample)]
                  [static_cast<std::size_t>(step)]
                      .vy;
        controls[static_cast<std::size_t>(step)].wz +=
            noises[static_cast<std::size_t>(sample)]
                  [static_cast<std::size_t>(step)]
                      .wz;
      }
      controls[static_cast<std::size_t>(step)] = applyAccelerationLimit(
          controls[static_cast<std::size_t>(step)], previous);
      previous = controls[static_cast<std::size_t>(step)];
    }

    bool collision = false;
    costs[static_cast<std::size_t>(sample)] =
        rolloutCost(controls, transformed_plan, cumulative_distances,
                    robot_pose_in_costmap, collision);
    if (!collision) {
      any_feasible = true;
    }
    min_cost = std::min(min_cost, costs[static_cast<std::size_t>(sample)]);
  }

  if (!any_feasible) {
    geometry_msgs::msg::TwistStamped stop;
    stop.header = pose.header;
    throw nav2_core::NoValidControl("MPPI found no collision-free trajectory");
  }

  std::vector<double> weights(static_cast<std::size_t>(batch_size_), 0.0);
  double weight_sum = 0.0;
  for (int sample = 0; sample < batch_size_; ++sample) {
    const double weight = std::exp(
        -std::min(700.0, (costs[static_cast<std::size_t>(sample)] - min_cost) /
                             temperature_));
    weights[static_cast<std::size_t>(sample)] = weight;
    weight_sum += weight;
  }
  if (weight_sum <= std::numeric_limits<double>::epsilon()) {
    weight_sum = 1.0;
  }

  for (int step = 0; step < time_steps_; ++step) {
    Control update{};
    for (int sample = 0; sample < batch_size_; ++sample) {
      const double normalized_weight =
          weights[static_cast<std::size_t>(sample)] / weight_sum;
      const auto &noise = noises[static_cast<std::size_t>(sample)]
                                [static_cast<std::size_t>(step)];
      update.vx += normalized_weight * noise.vx;
      update.vy += normalized_weight * noise.vy;
      update.wz += normalized_weight * noise.wz;
    }
    nominal_controls_[static_cast<std::size_t>(step)] = clampControl(Control{
        nominal_controls_[static_cast<std::size_t>(step)].vx + update.vx,
        nominal_controls_[static_cast<std::size_t>(step)].vy + update.vy,
        nominal_controls_[static_cast<std::size_t>(step)].wz + update.wz});
  }

  Control command = nominal_controls_.front();
  command.vx *= speed_limit_factor_;
  command.vy *= speed_limit_factor_;
  command.wz *= speed_limit_factor_;

  std::move(nominal_controls_.begin() + 1, nominal_controls_.end(),
            nominal_controls_.begin());
  nominal_controls_.back() = nominal_controls_[nominal_controls_.size() - 2];

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = command.vx;
  cmd_vel.twist.linear.y = command.vy;
  cmd_vel.twist.angular.z = command.wz;
  return cmd_vel;
}

nav_msgs::msg::Path OmniMppiController::transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped &pose) {
  if (global_plan_.poses.empty()) {
    throw nav2_core::InvalidPath("Received plan with zero length");
  }

  geometry_msgs::msg::PoseStamped robot_in_plan_frame;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_in_plan_frame)) {
    throw nav2_core::ControllerTFError(
        "Unable to transform robot pose into global plan frame");
  }

  std::size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::max();
  double accumulated_distance = 0.0;
  for (std::size_t i = 0; i < global_plan_.poses.size(); ++i) {
    if (i > 0) {
      accumulated_distance +=
          euclidean_distance(global_plan_.poses[i - 1], global_plan_.poses[i]);
    }
    if (accumulated_distance > max_robot_pose_search_dist_) {
      break;
    }
    const double distance =
        euclidean_distance(robot_in_plan_frame, global_plan_.poses[i]);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = i;
    }
  }

  const double max_extent =
      std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY()) /
      2.0;
  std::size_t end_index = closest_index + 1;
  while (end_index < global_plan_.poses.size() &&
         euclidean_distance(robot_in_plan_frame,
                            global_plan_.poses[end_index]) <= max_extent) {
    ++end_index;
  }
  end_index = std::max(end_index,
                       std::min(global_plan_.poses.size(), closest_index + 2));

  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = pose.header.stamp;
  transformed_plan.poses.reserve(end_index - closest_index);
  for (std::size_t i = closest_index; i < end_index; ++i) {
    geometry_msgs::msg::PoseStamped plan_pose = global_plan_.poses[i];
    plan_pose.header.stamp = robot_in_plan_frame.header.stamp;
    geometry_msgs::msg::PoseStamped local_pose;
    if (!transformPose(costmap_ros_->getBaseFrameID(), plan_pose, local_pose)) {
      throw nav2_core::ControllerTFError(
          "Unable to transform global plan pose into base frame");
    }
    local_pose.pose.position.z = 0.0;
    transformed_plan.poses.push_back(local_pose);
  }

  global_plan_.poses.erase(global_plan_.poses.begin(),
                           global_plan_.poses.begin() + closest_index);
  if (transformed_plan.poses.empty()) {
    throw nav2_core::InvalidPath("Resulting MPPI plan has zero poses");
  }
  return transformed_plan;
}

bool OmniMppiController::transformPose(
    const std::string &frame, const geometry_msgs::msg::PoseStamped &in_pose,
    geometry_msgs::msg::PoseStamped &out_pose) const {
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }
  try {
    tf_->transform(in_pose, out_pose, frame,
                   tf2::durationFromSec(transform_tolerance_));
    return true;
  } catch (const tf2::TransformException &exception) {
    RCLCPP_ERROR(logger_, "MPPI transform failed: %s", exception.what());
    return false;
  }
}

geometry_msgs::msg::PoseStamped OmniMppiController::referenceAtDistance(
    const nav_msgs::msg::Path &plan,
    const std::vector<double> &cumulative_distances, double distance) const {
  if (plan.poses.size() == 1 || cumulative_distances.back() <= 1e-6) {
    return plan.poses.back();
  }
  distance = std::clamp(distance, 0.0, cumulative_distances.back());
  auto upper = std::lower_bound(cumulative_distances.begin(),
                                cumulative_distances.end(), distance);
  const std::size_t upper_index = static_cast<std::size_t>(
      std::distance(cumulative_distances.begin(), upper));
  if (upper_index == 0) {
    return plan.poses.front();
  }
  if (upper_index >= plan.poses.size()) {
    return plan.poses.back();
  }
  const std::size_t lower_index = upper_index - 1;
  const double segment =
      cumulative_distances[upper_index] - cumulative_distances[lower_index];
  const double ratio =
      segment > 1e-6 ? (distance - cumulative_distances[lower_index]) / segment
                     : 1.0;
  geometry_msgs::msg::PoseStamped result = plan.poses[lower_index];
  result.pose.position.x += ratio * (plan.poses[upper_index].pose.position.x -
                                     plan.poses[lower_index].pose.position.x);
  result.pose.position.y += ratio * (plan.poses[upper_index].pose.position.y -
                                     plan.poses[lower_index].pose.position.y);
  result.pose.orientation = tf2::toMsg(tf2::Quaternion(
      tf2::Vector3(0.0, 0.0, 1.0),
      lerpAngle(tf2::getYaw(plan.poses[lower_index].pose.orientation),
                tf2::getYaw(plan.poses[upper_index].pose.orientation), ratio)));
  return result;
}

OmniMppiController::Control
OmniMppiController::clampControl(const Control &control) const {
  return Control{std::clamp(control.vx, vx_min_, vx_max_),
                 std::clamp(control.vy, vy_min_, vy_max_),
                 std::clamp(control.wz, wz_min_, wz_max_)};
}

OmniMppiController::Control
OmniMppiController::applyAccelerationLimit(const Control &control,
                                           const Control &previous) const {
  return clampControl(
      Control{std::clamp(control.vx, previous.vx - max_accel_x_ * model_dt_,
                         previous.vx + max_accel_x_ * model_dt_),
              std::clamp(control.vy, previous.vy - max_accel_y_ * model_dt_,
                         previous.vy + max_accel_y_ * model_dt_),
              std::clamp(control.wz, previous.wz - max_accel_theta_ * model_dt_,
                         previous.wz + max_accel_theta_ * model_dt_)});
}

double OmniMppiController::rolloutCost(
    const std::vector<Control> &controls, const nav_msgs::msg::Path &plan,
    const std::vector<double> &cumulative_distances,
    const geometry_msgs::msg::PoseStamped &robot_pose_in_costmap,
    bool &collision) const {
  State state;
  Control previous = controls.front();
  const double reference_speed =
      std::max(0.15, std::hypot(controls.front().vx, controls.front().vy));
  double cost = 0.0;
  collision = false;

  for (std::size_t step = 0; step < controls.size(); ++step) {
    const auto &control = controls[step];
    state.x +=
        (control.vx * std::cos(state.yaw) - control.vy * std::sin(state.yaw)) *
        model_dt_;
    state.y +=
        (control.vx * std::sin(state.yaw) + control.vy * std::cos(state.yaw)) *
        model_dt_;
    state.yaw = normalizeAngle(state.yaw + control.wz * model_dt_);

    const double reference_distance =
        reference_speed * model_dt_ * static_cast<double>(step + 1);
    const auto reference =
        referenceAtDistance(plan, cumulative_distances, reference_distance);
    const double position_error =
        std::hypot(state.x - reference.pose.position.x,
                   state.y - reference.pose.position.y);
    const double heading_error =
        normalizeAngle(state.yaw - tf2::getYaw(reference.pose.orientation));
    const bool terminal = step + 1 == controls.size();
    const double position_weight =
        terminal ? terminal_path_weight_ : path_weight_;
    const double heading_weight =
        terminal ? terminal_heading_weight_ : heading_weight_;
    cost += position_weight * sqr(position_error) +
            heading_weight * sqr(heading_error);
    cost += control_weight_ * (sqr(control.vx / std::max(1e-3, vx_max_)) +
                               sqr(control.vy / std::max(1e-3, vy_max_)) +
                               sqr(control.wz / std::max(1e-3, wz_max_)));
    cost += smoothness_weight_ *
            (sqr(control.vx - previous.vx) + sqr(control.vy - previous.vy) +
             sqr(control.wz - previous.wz));
    previous = control;

    bool outside_costmap = false;
    const auto cell_cost =
        costAtState(state, robot_pose_in_costmap, outside_costmap);
    if (outside_costmap || cell_cost >= collision_cost_threshold_ ||
        (consider_unknown_as_collision_ &&
         cell_cost == nav2_costmap_2d::NO_INFORMATION)) {
      collision = true;
      cost += collision_weight_;
    } else {
      cost += collision_weight_ *
              sqr(static_cast<double>(cell_cost) /
                  static_cast<double>(std::max(
                      1, static_cast<int>(collision_cost_threshold_))));
    }
  }

  const auto &goal = plan.poses.back().pose;
  cost += terminal_path_weight_ *
          sqr(std::hypot(state.x - goal.position.x, state.y - goal.position.y));
  cost += terminal_heading_weight_ *
          sqr(normalizeAngle(state.yaw - tf2::getYaw(goal.orientation)));
  return cost;
}

unsigned char OmniMppiController::costAtState(
    const State &state,
    const geometry_msgs::msg::PoseStamped &robot_pose_in_costmap,
    bool &outside_costmap) const {
  const double robot_yaw = tf2::getYaw(robot_pose_in_costmap.pose.orientation);
  const double world_x = robot_pose_in_costmap.pose.position.x +
                         std::cos(robot_yaw) * state.x -
                         std::sin(robot_yaw) * state.y;
  const double world_y = robot_pose_in_costmap.pose.position.y +
                         std::sin(robot_yaw) * state.x +
                         std::cos(robot_yaw) * state.y;
  unsigned int map_x = 0;
  unsigned int map_y = 0;
  if (!costmap_->worldToMap(world_x, world_y, map_x, map_y)) {
    outside_costmap = true;
    return nav2_costmap_2d::NO_INFORMATION;
  }
  outside_costmap = false;
  return costmap_->getCost(map_x, map_y);
}

double OmniMppiController::normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double OmniMppiController::lerpAngle(double from, double to, double ratio) {
  return normalizeAngle(from + ratio * normalizeAngle(to - from));
}

rcl_interfaces::msg::SetParametersResult
OmniMppiController::dynamicParametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::lock_guard<std::mutex> lock(mutex_);

  const auto prefix = plugin_name_ + ".";
  for (const auto &parameter : parameters) {
    const auto &name = parameter.get_name();
    if (name.rfind(prefix, 0) != 0) {
      continue;
    }
    const auto suffix = name.substr(prefix.size());
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      const double value = parameter.as_double();
      if (!std::isfinite(value)) {
        result.successful = false;
        result.reason = "MPPI parameters must be finite";
        return result;
      }
      if (suffix == "transform_tolerance")
        transform_tolerance_ = value;
      else if (suffix == "model_dt")
        model_dt_ = value;
      else if (suffix == "temperature")
        temperature_ = value;
      else if (suffix == "noise_std_x")
        noise_std_x_ = value;
      else if (suffix == "noise_std_y")
        noise_std_y_ = value;
      else if (suffix == "noise_std_theta")
        noise_std_theta_ = value;
      else if (suffix == "vx_min")
        vx_min_ = value;
      else if (suffix == "vx_max")
        vx_max_ = value;
      else if (suffix == "vy_min")
        vy_min_ = value;
      else if (suffix == "vy_max")
        vy_max_ = value;
      else if (suffix == "wz_min")
        wz_min_ = value;
      else if (suffix == "wz_max")
        wz_max_ = value;
      else if (suffix == "max_accel_x")
        max_accel_x_ = value;
      else if (suffix == "max_accel_y")
        max_accel_y_ = value;
      else if (suffix == "max_accel_theta")
        max_accel_theta_ = value;
      else if (suffix == "path_weight")
        path_weight_ = value;
      else if (suffix == "heading_weight")
        heading_weight_ = value;
      else if (suffix == "terminal_path_weight")
        terminal_path_weight_ = value;
      else if (suffix == "terminal_heading_weight")
        terminal_heading_weight_ = value;
      else if (suffix == "control_weight")
        control_weight_ = value;
      else if (suffix == "smoothness_weight")
        smoothness_weight_ = value;
      else if (suffix == "collision_weight")
        collision_weight_ = value;
      else if (suffix == "max_robot_pose_search_dist")
        max_robot_pose_search_dist_ = value;
    } else if (parameter.get_type() ==
               rclcpp::ParameterType::PARAMETER_INTEGER) {
      const int value = static_cast<int>(parameter.as_int());
      if (suffix == "time_steps")
        time_steps_ = value;
      else if (suffix == "batch_size")
        batch_size_ = value;
      else if (suffix == "collision_cost_threshold")
        collision_cost_threshold_ =
            static_cast<unsigned char>(std::clamp(value, 1, 255));
    } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL &&
               suffix == "consider_unknown_as_collision") {
      consider_unknown_as_collision_ = parameter.as_bool();
    }
  }

  if (time_steps_ < 2 || batch_size_ < 2 || model_dt_ <= 0.0 ||
      temperature_ <= 0.0 || vx_min_ > vx_max_ || vy_min_ > vy_max_ ||
      wz_min_ > wz_max_ || max_accel_x_ <= 0.0 || max_accel_y_ <= 0.0 ||
      max_accel_theta_ <= 0.0) {
    result.successful = false;
    result.reason = "Invalid MPPI parameter relation";
    return result;
  }
  nominal_controls_.assign(static_cast<std::size_t>(time_steps_), Control{});
  nominal_initialized_ = false;
  return result;
}

// clang-format off
}  // namespace pb_omni_mppi_controller
// clang-format on

PLUGINLIB_EXPORT_CLASS(pb_omni_mppi_controller::OmniMppiController,
                       nav2_core::Controller)
