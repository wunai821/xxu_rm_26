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
 * MPPI（Model Predictive Path Integral）不是单独运行的 ROS 节点，
 * 而是通过 Nav2 的 nav2_core::Controller 接口由 controller_server 调用。
 * 每个控制周期都会执行以下流程：
 * 1. 把全局路径裁剪并转换到机器人 base frame；
 * 2. 以当前控制序列为中心，对 vx、vy、wz 添加随机扰动；
 * 3. 使用全向底盘运动学模型向前滚动预测每条候选轨迹；
 * 4. 根据路径误差、朝向误差、控制平滑性和代价地图代价计算轨迹代价；
 * 5. 用代价对应的权重更新控制序列，并输出序列第一个控制量。
 *
 * 其中 vx、vy、wz 均表示机器人基座坐标系下的速度：
 * - vx：前后方向线速度；
 * - vy：左右方向线速度；
 * - wz：绕 Z 轴的角速度。
 */
class OmniMppiController : public nav2_core::Controller {
public:
  OmniMppiController() = default;
  ~OmniMppiController() override = default;

  // Nav2 生命周期配置：读取参数、保存 TF/代价地图对象并初始化控制序列。
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // 释放发布者、参数回调、路径和代价地图等资源。
  void cleanup() override;

  // 激活动态参数回调，使运行中的 ros2 param set 可以更新 MPPI 参数。
  void activate() override;

  // 停止接受动态参数更新，控制器进入非活动状态。
  void deactivate() override;

  // Nav2 每个控制周期调用一次，返回当前周期要执行的 TwistStamped。
  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                          const geometry_msgs::msg::Twist &velocity,
                          nav2_core::GoalChecker *goal_checker) override;

  // 接收规划器生成的新全局路径，并清空上一条路径对应的名义控制序列。
  void setPlan(const nav_msgs::msg::Path &path) override;

  // 接收 Nav2 外部速度限制。percentage=true 时 speed_limit 为百分比。
  void setSpeedLimit(const double &speed_limit,
                     const bool &percentage) override;

private:
  struct Control {
    // 单个离散预测时刻的控制输入，坐标系为机器人 base frame。
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
  };

  struct State {
    // 相对于当前机器人初始状态的预测位姿，x/y 单位为米，yaw 单位为弧度。
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  // 查找路径最近点，裁剪已通过的部分，并将剩余路径转换到 base frame。
  nav_msgs::msg::Path
  transformGlobalPlan(const geometry_msgs::msg::PoseStamped &pose);

  // 使用 TF 将位姿转换到目标坐标系；失败时返回 false，由上层抛出 Nav2 异常。
  bool transformPose(const std::string &frame,
                     const geometry_msgs::msg::PoseStamped &in_pose,
                     geometry_msgs::msg::PoseStamped &out_pose) const;

  // 按路径累计距离插值得到预测时刻对应的参考位姿。
  geometry_msgs::msg::PoseStamped
  referenceAtDistance(const nav_msgs::msg::Path &plan,
                      const std::vector<double> &cumulative_distances,
                      double distance) const;

  // 将速度限制在 vx/vy/wz 的上下界内。
  Control clampControl(const Control &control) const;

  // 限制相邻预测时刻的速度变化，避免采样出不可实现的突变控制量。
  Control applyAccelerationLimit(const Control &control,
                                 const Control &previous) const;

  // 用运动学模型滚动预测一条控制序列，并计算综合代价。
  double
  rolloutCost(const std::vector<Control> &controls,
              const nav_msgs::msg::Path &plan,
              const std::vector<double> &cumulative_distances,
              const geometry_msgs::msg::PoseStamped &robot_pose_in_costmap,
              bool &collision) const;

  // 将预测状态转换到 costmap 坐标系，查询对应栅格的障碍物代价。
  unsigned char
  costAtState(const State &state,
              const geometry_msgs::msg::PoseStamped &robot_pose_in_costmap,
              bool &outside_costmap) const;

  // 动态参数回调：校验参数关系后更新成员变量，并重置名义控制序列。
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  // 将角度归一化到 [-pi, pi]，避免航向误差在 pi 附近跳变。
  static double normalizeAngle(double angle);

  // 沿最短角度方向对两个航向角做线性插值。
  static double lerpAngle(double from, double to, double ratio);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("omni_mppi_controller")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D *costmap_{nullptr};
  nav_msgs::msg::Path global_plan_;
  std::string plugin_name_;
  // TF 查询容差以及预测离散化参数。
  double transform_tolerance_{0.2};
  int time_steps_{15};
  int batch_size_{256};
  double model_dt_{0.1};
  double temperature_{0.35};

  // 采样噪声标准差。噪声越大，探索范围越大，但计算结果可能更抖动。
  double noise_std_x_{0.35};
  double noise_std_y_{0.35};
  double noise_std_theta_{0.25};

  // 速度和加速度约束，必须与底盘实际可执行范围保持一致。
  double vx_min_{-1.5};
  double vx_max_{1.5};
  double vy_min_{-1.5};
  double vy_max_{1.5};
  double wz_min_{-0.8};
  double wz_max_{0.8};
  double max_accel_x_{0.8};
  double max_accel_y_{0.8};
  double max_accel_theta_{1.5};

  // 代价函数权重：路径/航向跟踪、终点误差、控制量、平滑性和障碍物代价。
  double path_weight_{8.0};
  double heading_weight_{1.5};
  double terminal_path_weight_{16.0};
  double terminal_heading_weight_{3.0};
  double control_weight_{0.04};
  double smoothness_weight_{0.12};
  double collision_weight_{120.0};
  // 达到该栅格代价时将候选轨迹标记为碰撞轨迹。
  unsigned char collision_cost_threshold_{
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE};
  bool consider_unknown_as_collision_{true};
  double max_robot_pose_search_dist_{2.0};

  // 控制器运行状态和 MPPI 的滚动名义控制序列。
  bool configured_{false};
  bool nominal_initialized_{false};
  std::vector<Control> nominal_controls_;
  // 固定种子便于复现实验；正式评测时可改为随机种子。
  std::mt19937 random_engine_{0x5EED2026U};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;
  // computeVelocityCommands 与动态参数回调可能并发执行，因此需要互斥保护。
  mutable std::mutex mutex_;
  double speed_limit_factor_{1.0};
};

// clang-format off
}  // namespace pb_omni_mppi_controller
// clang-format on

// clang-format off
#endif  // PB_OMNI_MPPI_CONTROLLER__OMNI_MPPI_CONTROLLER_HPP_
// clang-format on
