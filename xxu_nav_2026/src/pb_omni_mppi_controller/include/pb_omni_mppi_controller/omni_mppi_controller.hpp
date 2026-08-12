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

#ifndef PB_OMNI_MPPI_CONTROLLER__OMNI_MPPI_CONTROLLER_HPP_
#define PB_OMNI_MPPI_CONTROLLER__OMNI_MPPI_CONTROLLER_HPP_

#include <cstddef>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace pb_omni_mppi_controller {

/**
 * @brief 全向底盘的轻量 MPPI 控制器。
 *
 * 控制量是机器人基座坐标系下的 vx、vy、wz。每个控制周期从上一周期的
 * 控制序列开始采样，并根据局部路径误差和代价地图代价更新该序列。
 */
class OmniMppiController : public nav2_core::Controller {
public:
  OmniMppiController() = default;
  ~OmniMppiController() override = default;

  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                          const geometry_msgs::msg::Twist &velocity,
                          nav2_core::GoalChecker *goal_checker) override;

  void setPlan(const nav_msgs::msg::Path &path) override;
  void setSpeedLimit(const double &speed_limit,
                     const bool &percentage) override;

private:
  struct Control {
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
  };

  struct State {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  nav_msgs::msg::Path
  transformGlobalPlan(const geometry_msgs::msg::PoseStamped &pose);

  bool transformPose(const std::string &frame,
                     const geometry_msgs::msg::PoseStamped &in_pose,
                     geometry_msgs::msg::PoseStamped &out_pose) const;

  geometry_msgs::msg::PoseStamped
  referenceAtDistance(const nav_msgs::msg::Path &plan,
                      const std::vector<double> &cumulative_distances,
                      double distance) const;

  Control clampControl(const Control &control) const;
  Control applyAccelerationLimit(const Control &control,
                                 const Control &previous) const;

  double
  rolloutCost(const std::vector<Control> &controls,
              const nav_msgs::msg::Path &plan,
              const std::vector<double> &cumulative_distances,
              const geometry_msgs::msg::PoseStamped &robot_pose_in_costmap,
              bool &collision) const;

  unsigned char
  costAtState(const State &state,
              const geometry_msgs::msg::PoseStamped &robot_pose_in_costmap,
              bool &outside_costmap) const;

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  static double normalizeAngle(double angle);
  static double lerpAngle(double from, double to, double ratio);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("omni_mppi_controller")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D *costmap_{nullptr};
  nav_msgs::msg::Path global_plan_;
  std::string plugin_name_;

  double transform_tolerance_{0.2};
  int time_steps_{15};
  int batch_size_{256};
  double model_dt_{0.1};
  double temperature_{0.35};
  double noise_std_x_{0.35};
  double noise_std_y_{0.35};
  double noise_std_theta_{0.25};

  double vx_min_{-1.5};
  double vx_max_{1.5};
  double vy_min_{-1.5};
  double vy_max_{1.5};
  double wz_min_{-0.8};
  double wz_max_{0.8};
  double max_accel_x_{0.8};
  double max_accel_y_{0.8};
  double max_accel_theta_{1.5};

  double path_weight_{8.0};
  double heading_weight_{1.5};
  double terminal_path_weight_{16.0};
  double terminal_heading_weight_{3.0};
  double control_weight_{0.04};
  double smoothness_weight_{0.12};
  double collision_weight_{120.0};
  unsigned char collision_cost_threshold_{
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE};
  bool consider_unknown_as_collision_{true};
  double max_robot_pose_search_dist_{2.0};

  bool configured_{false};
  bool nominal_initialized_{false};
  std::vector<Control> nominal_controls_;
  std::mt19937 random_engine_{0x5EED2026U};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;
  mutable std::mutex mutex_;
  double speed_limit_factor_{1.0};
};

// clang-format off
}  // namespace pb_omni_mppi_controller
// clang-format on

// clang-format off
#endif  // PB_OMNI_MPPI_CONTROLLER__OMNI_MPPI_CONTROLLER_HPP_
// clang-format on
