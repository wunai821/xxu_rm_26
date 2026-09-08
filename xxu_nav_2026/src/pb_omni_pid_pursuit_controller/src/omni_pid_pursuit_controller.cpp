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

#include "pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;
using namespace nav2_costmap_2d;  // NOLINT
using rcl_interfaces::msg::ParameterType;

namespace pb_omni_pid_pursuit_controller
{

/**
 * @brief 控制器配置 —— 生命周期配置阶段
 *
 * 初始化所有资源：保存节点引用、TF 缓冲区、代价地图；声明并读取所有
 * 控制参数；创建 PID 控制器；创建可视化发布者（局部路径、胡萝卜点、曲率点）。
 *
 * 参数通过 ROS2 参数系统管理，使用 declare_parameter_if_not_declared
 * 保证幂等（重复声明不会报错），参数名使用 "插件名.参数名" 的命名空间格式。
 *
 * @param parent     生命周期节点的弱引用指针
 * @param name       插件名称（用于参数命名空间隔离）
 * @param tf         TF2 坐标变换缓冲区
 * @param costmap_ros 代价地图 ROS 包装器
 * @throws nav2_core::ControllerException 如果无法锁定节点
 */
void OmniPidPursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double transform_tolerance = 1.0;
  double control_frequency = 20.0;
  max_robot_pose_search_dist_ = getCostmapMaxExtent();

  // ========== 声明并读取所有控制参数 ==========

  // --- 平移 PID 参数（控制 X/Y 方向的线速度） ---
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_kp", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_ki", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_kd", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_rotation", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_kp", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_ki", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_kd", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_max_sum_error", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_velocity_scaled_lookahead_dist", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_interpolation", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading_treshold", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_min", rclcpp::ParameterValue(-3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_linear_max", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_angular_min", rclcpp::ParameterValue(-3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_angular_max", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist",
    rclcpp::ParameterValue(getCostmapMaxExtent()));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_min", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_max", rclcpp::ParameterValue(0.7));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".reduction_ratio_at_high_curvature", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_forward_dist", rclcpp::ParameterValue(0.7));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_backward_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_velocity_scaling_factor_rate", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_path_smoothing", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".path_smoothing_iterations", rclcpp::ParameterValue(1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".path_smoothing_max_offset", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_lookahead_dist", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_sample_dist", rclcpp::ParameterValue(0.15));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lateral_accel", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_max_deceleration", rclcpp::ParameterValue(1.0));

  // 从参数服务器读取各项参数值到成员变量
  node->get_parameter(plugin_name_ + ".translation_kp", translation_kp_);
  node->get_parameter(plugin_name_ + ".translation_ki", translation_ki_);
  node->get_parameter(plugin_name_ + ".translation_kd", translation_kd_);
  node->get_parameter(plugin_name_ + ".enable_rotation", enable_rotation_);
  node->get_parameter(plugin_name_ + ".rotation_kp", rotation_kp_);
  node->get_parameter(plugin_name_ + ".rotation_ki", rotation_ki_);
  node->get_parameter(plugin_name_ + ".rotation_kd", rotation_kd_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(plugin_name_ + ".min_max_sum_error", min_max_sum_error_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(
    plugin_name_ + ".use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(plugin_name_ + ".use_interpolation", use_interpolation_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node->get_parameter(
    plugin_name_ + ".use_rotate_to_heading_treshold", use_rotate_to_heading_treshold_);
  node->get_parameter(
    plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
  node->get_parameter(
    plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  // 如果接近减速距离超过了代价地图前方范围的一半，机器人会长期处于减速状态
  if (approach_velocity_scaling_dist_ > costmap_->getSizeInMetersX() / 2.0) {
    RCLCPP_WARN(
      logger_,
      "approach_velocity_scaling_dist is larger than forward costmap extent, "
      "leading to permanent slowdown");
  }
  node->get_parameter(plugin_name_ + ".v_linear_max", v_linear_max_);
  node->get_parameter(plugin_name_ + ".v_linear_min", v_linear_min_);
  node->get_parameter(plugin_name_ + ".v_angular_max", v_angular_max_);
  node->get_parameter(plugin_name_ + ".v_angular_min", v_angular_min_);
  node->get_parameter(plugin_name_ + ".max_robot_pose_search_dist", max_robot_pose_search_dist_);
  node->get_parameter(plugin_name_ + ".curvature_min", curvature_min_);
  node->get_parameter(plugin_name_ + ".curvature_max", curvature_max_);
  node->get_parameter(
    plugin_name_ + ".reduction_ratio_at_high_curvature", reduction_ratio_at_high_curvature_);
  node->get_parameter(plugin_name_ + ".curvature_forward_dist", curvature_forward_dist_);
  node->get_parameter(plugin_name_ + ".curvature_backward_dist", curvature_backward_dist_);
  node->get_parameter(
    plugin_name_ + ".max_velocity_scaling_factor_rate", max_velocity_scaling_factor_rate_);
  node->get_parameter(plugin_name_ + ".use_path_smoothing", use_path_smoothing_);
  node->get_parameter(plugin_name_ + ".path_smoothing_iterations", path_smoothing_iterations_);
  node->get_parameter(plugin_name_ + ".path_smoothing_max_offset", path_smoothing_max_offset_);
  node->get_parameter(plugin_name_ + ".curvature_lookahead_dist", curvature_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".curvature_sample_dist", curvature_sample_dist_);
  node->get_parameter(plugin_name_ + ".max_lateral_accel", max_lateral_accel_);
  node->get_parameter(
    plugin_name_ + ".curvature_max_deceleration", curvature_max_deceleration_);

  node->get_parameter("controller_frequency", control_frequency);
  last_velocity_scaling_factor_ = v_linear_max_;

  // 将 transform_tolerance 从秒转换为 tf2::Duration 类型
  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  // 控制周期 = 1 / 控制频率（例如 20Hz -> 0.05秒）
  control_duration_ = 1.0 / control_frequency;

  // 创建发布者：局部路径、胡萝卜点、曲率可视化 marker
  local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  curvature_points_pub_ =
    node_.lock()
      ->create_publisher<visualization_msgs::msg::MarkerArray>(
        "curvature_points_marker_array", rclcpp::QoS(10));

  // 创建平移 PID 控制器：控制到前视点的距离 -> X/Y 方向线速度
  move_pid_ = std::make_shared<PID>(
    control_duration_, v_linear_max_, v_linear_min_, translation_kp_, translation_kd_,
    translation_ki_);
  move_pid_->setIntegralLimit(min_max_sum_error_);
  // 创建旋转 PID 控制器：控制到目标朝向的角度差 -> Z 轴角速度
  heading_pid_ = std::make_shared<PID>(
    control_duration_, v_angular_max_, v_angular_min_, rotation_kp_, rotation_kd_, rotation_ki_);
  heading_pid_->setIntegralLimit(min_max_sum_error_);
}

/**
 * @brief 清理控制器 —— 释放所有发布者资源
 */
void OmniPidPursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " pb_omni_pid_pursuit_controller::OmniPidPursuitController",
    plugin_name_.c_str());
  local_path_pub_.reset();
  carrot_pub_.reset();
  curvature_points_pub_.reset();
}

/**
 * @brief 激活控制器 —— 激活发布者并注册动态参数回调
 *
 * 在生命周期 activate 状态转换时被调用。
 * 激活后控制器可以接收 computeVelocityCommands 调用。
 */
void OmniPidPursuitController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "pb_omni_pid_pursuit_controller::OmniPidPursuitController",
    plugin_name_.c_str());
  local_path_pub_->on_activate();
  carrot_pub_->on_activate();
  curvature_points_pub_->on_activate();
  // 注册动态参数回调：允许运行时通过 rqt_reconfigure 等工具调整参数
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&OmniPidPursuitController::dynamicParametersCallback, this, std::placeholders::_1));
}

/**
 * @brief 停用控制器 —— 停用发布者并移除参数回调
 */
void OmniPidPursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "pb_omni_pid_pursuit_controller::OmniPidPursuitController",
    plugin_name_.c_str());
  local_path_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
  curvature_points_pub_->on_deactivate();
  dyn_params_handler_.reset();
}

/**
 * @brief 计算速度指令 —— 控制器核心循环
 *
 * 此函数被 Nav2 以固定频率（通常 20Hz）调用，每轮执行以下步骤：
 *
 *   1. 获取代价地图互斥锁，确保地图数据一致性
 *   2. 将全局路径变换到机器人基座坐标系 (base_link)
 *   3. 根据当前速度确定前视距离（速度快看得远，速度慢看得近）
 *   4. 在局部路径上找到前视点（胡萝卜点）
 *   5. 用平移 PID 根据前视点距离计算线速度大小
 *   6. 用旋转 PID 根据前视点朝向计算角速度
 *   7. 基于路径曲率限制速度（弯道减速）
 *   8. 基于接近目标程度减速（末端平滑停止）
 *   9. 碰撞检测：对路径采样点在代价地图中检测障碍物
 *  10. 将线速度按前视点方向分解为 X 和 Y 分量
 *
 * 注意：由于是全向轮底盘，线速度和角速度是独立控制的。
 *       线速度分解为 X 和 Y 两个分量让机器人可以朝着前视点的方向直接移动。
 *
 * @param pose         机器人当前位姿（全局坐标系）
 * @param velocity     机器人当前速度
 * @param goal_checker 目标检查器（本控制器未使用）
 * @return             速度指令（包含时间戳）
 * @throws nav2_core::NoValidControl 如果路径上检测到碰撞
 */
geometry_msgs::msg::TwistStamped OmniPidPursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  // 获取代价地图及其互斥锁，确保整个计算过程中地图数据不被更新
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // 步骤1：将全局路径变换到机器人基座坐标系
  auto transformed_plan = transformGlobalPlan(pose);

  // 步骤2：计算前视距离，在路径上找到前视点（胡萝卜点），并发布可视化
  double lookahead_dist = getLookAheadDistance(velocity);

  // Smooth only the geometric reference. Collision checking below deliberately
  // continues to use the planner's original path, so smoothing cannot cut a
  // corner through an obstacle.
  const auto pursuit_plan = smoothPath(transformed_plan);
  auto carrot_pose = getLookAheadPoint(lookahead_dist, pursuit_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  // 步骤3：计算到前视点的直线距离和方向角
  // lin_dist: 机器人原点到前视点的欧几里得距离
  // theta_dist: 前视点相对于机器人前方（X 轴正向）的角度
  double lin_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  double theta_dist = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  double angle_to_goal = tf2::getYaw(carrot_pose.pose.orientation);

  // 步骤4：先旋转到位再前进模式
  // 如果启用了"先转向目标朝向"，且朝向误差大于阈值：
  //   - 将 lin_dist 设为0，机器人原地旋转直到对准目标朝向
  //   - 对准后再恢复正常的平移控制
  if (use_rotate_to_heading_) {
    angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    if (fabs(angle_to_goal) > use_rotate_to_heading_treshold_) {
      lin_dist = 0;
    }
  }

  // 步骤5+6：PID 控制
  // 平移 PID：输入=距离误差，期望=0（我们希望距离为0即到达前视点），输出=线速度大小
  auto lin_vel = move_pid_->calculate(lin_dist, 0);
  // 旋转 PID：如果启用旋转控制则计算角速度，否则为0（纯平移模式）
  auto angular_vel = enable_rotation_ ? heading_pid_->calculate(angle_to_goal, 0) : 0.0;

  // 步骤7：基于路径曲率限制速度（弯道自动减速）
  applyCurvatureLimitation(pursuit_plan, carrot_pose, lin_vel);

  // 步骤8：接近目标时减速（平滑停止）
  applyApproachVelocityScaling(transformed_plan, lin_vel);

  // 步骤9：碰撞检测 —— 将局部路径采样点变换到代价地图坐标系进行检查
  nav_msgs::msg::Path costmap_frame_local_plan;

  int sample_points = 10;
  int plan_size = transformed_plan.poses.size();
  for (int i = 0; i < sample_points; ++i) {
    // 在路径上均匀采样 10 个点进行碰撞检查
    int index = std::min((i * plan_size) / sample_points, plan_size - 1);
    geometry_msgs::msg::PoseStamped map_pose;
    transformPose(costmap_ros_->getGlobalFrameID(), transformed_plan.poses[index], map_pose);
    costmap_frame_local_plan.poses.push_back(map_pose);
  }

  // 步骤10：组装速度指令
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;

  if (!isCollisionDetected(costmap_frame_local_plan)) {
    // 无碰撞：将线速度沿前视点方向分解为 X 和 Y 分量
    // lin_vel * cos(theta_dist) -> 前视点在机器人前方方向的速度分量
    // lin_vel * sin(theta_dist) -> 前视点在机器人侧方方向的速度分量
    // 这种分解让全向轮底盘可以直接朝前视点方向移动，而不需要先转向
    cmd_vel.twist.linear.x = lin_vel * cos(theta_dist);
    cmd_vel.twist.linear.y = lin_vel * sin(theta_dist);
    cmd_vel.twist.angular.z = angular_vel;
  } else {
    // 检测到碰撞：抛出异常，让 Nav2 行为树进入恢复状态（如原地旋转、重新规划等）
    throw nav2_core::NoValidControl("Collision detected in the trajectory. Stopping the robot!");
  }

  return cmd_vel;
}

void OmniPidPursuitController::setPlan(const nav_msgs::msg::Path & path) { global_plan_ = path; }

/**
 * @brief 设置速度限制（未实现）
 *
 * 本控制器暂不支持外部速度限制功能，调用时会打印警告。
 */
void OmniPidPursuitController::setSpeedLimit(
  const double & /*speed_limit*/, const bool & /*percentage*/)
{
  RCLCPP_WARN(logger_, "Speed limit is not implemented in this controller.");
}

/**
 * @brief 将全局路径变换到机器人基座坐标系并裁剪
 *
 * 完整流程：
 *   1. 将机器人位姿变换到全局路径的坐标系（通常是 map 或 odom）
 *   2. 在搜索范围内找到路径上离机器人最近的点
 *   3. 将该最近点之后、代价地图范围内的位姿都变换到 base_link 坐标系
 *   4. 从全局路径中删除已走过的部分（路径剪枝），减小后续计算量
 *
 * 路径剪枝的意义：每轮控制循环后，机器人向前移动了一段距离，
 * 路径上已经经过的点没有保留价值，删除它们可以加快下一轮
 * 的最近点查找和路径变换速度。
 *
 * @param pose 机器人当前在全局坐标系中的位姿
 * @return     在 base_link 坐标系下的局部路径
 * @throws nav2_core::InvalidPath       如果全局路径为空
 * @throws nav2_core::ControllerTFError 如果坐标变换失败
 */
nav_msgs::msg::Path OmniPidPursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::InvalidPath("Received plan with zero length");
  }

  // 将机器人位姿变换到全局路径所在的坐标系
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::ControllerTFError("Unable to transform robot pose into global plan's frame");
  }

  // 代价地图的最大范围（半径），用于确定需要变换多少路径点
  double max_costmap_extent = getCostmapMaxExtent();

  // 限定搜索范围：只在路径前方 max_robot_pose_search_dist 内搜索最近点
  auto closest_pose_upper_bound = nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  // 在搜索范围内找到离机器人位姿最近的路径点
  // 使用 min_by 算法，以欧几里得距离为比较标准
  // 截断到 closest_pose_upper_bound 是为了避免路径有折返时
  // 错误地匹配到路径后面离得更近的点
  auto transformation_begin = nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // 从最近点开始，收集到代价地图边界范围内的所有路径点
  // 超出代价地图范围的点不做变换（碰撞检测用不到）
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) { return euclidean_distance(pose, robot_pose) > max_costmap_extent; });

  // Lambda: 将全局坐标系下的位姿变换到机器人 base_link 坐标系
  // 变换后 z 坐标强制设为 0，因为机器人只在地面平面运动
  auto transform_global_pose_to_local = [&](const auto & global_plan_pose) {
    geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
    stamped_pose.header.frame_id = global_plan_.header.frame_id;
    stamped_pose.header.stamp = robot_pose.header.stamp;
    stamped_pose.pose = global_plan_pose.pose;
    transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
    transformed_pose.pose.position.z = 0.0;
    return transformed_pose;
  };

  // 批量变换：将 [transformation_begin, transformation_end) 范围内的点全部变换
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end, std::back_inserter(transformed_plan.poses),
    transform_global_pose_to_local);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // 路径剪枝：从全局路径中删除已经经过的部分
  // 下一次循环时不再处理这些点，提高效率
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  local_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::InvalidPath("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

/**
 * @brief 创建胡萝卜点（前视点）的可视化消息
 *
 * 发布一个 PointStamped 消息，在 RViz 中显示为一个点状标记。
 * z 坐标设为 0.01 使标记略高于地面，便于观察。
 *
 * @param carrot_pose 前视点（已在 base_link 坐标系中）
 * @return            PointStamped 消息的独占指针
 */
std::unique_ptr<geometry_msgs::msg::PointStamped> OmniPidPursuitController::createCarrotMsg(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;  // 略高于地面，在 RViz 中更显眼
  return carrot_msg;
}

/**
 * @brief 在局部路径上查找前视点（Lookahead Point）
 *
 * 纯追踪算法的核心步骤：
 *   1. 从路径起点（机器人位置）开始遍历
 *   2. 找到第一个距离 >= 前视距离的点作为目标
 *   3. 如果启用插值，精确计算该线段上恰好距离 = 前视距离的交点
 *
 * 插值模式的好处：获得平滑连续的前视点位置，避免前视点在路径离散点上跳跃。
 *
 * @param lookahead_dist    前视距离（米）
 * @param transformed_plan  已经在机器人坐标系下的局部路径
 * @return                  前视点的位姿
 */
geometry_msgs::msg::PoseStamped OmniPidPursuitController::getLookAheadPoint(
  const double & lookahead_dist, const nav_msgs::msg::Path & transformed_plan)
{
  // 找到第一个距离原点（机器人位置）大于等于前视距离的路径点
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  // 如果所有路径点都在前视距离以内，取最后一个点作为目标
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  } else if (use_interpolation_ && goal_pose_it != transformed_plan.poses.begin()) {
    // 插值模式：前一个点在圆内，当前点在圆外（或圆上）
    // 圆与线段之间存在唯一交点，通过解析几何公式精确计算交点位置
    // 这样可以获得平滑的前视点，避免在离散路径点上跳跃
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position, goal_pose_it->pose.position, lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    return pose;
  }

  return *goal_pose_it;
}

/**
 * @brief 计算圆与线段的交点（圆心在原点）
 *
 * 前视点插值的数学基础：
 *   所有位姿已经变换到机器人坐标系（原点在机器人位置），
 *   前视距离为半径 R，需要找到线段上距离原点恰好为 R 的点。
 *
 * 使用标准圆-线交点公式：
 *   线段由两点 P1(x1,y1), P2(x2,y2) 定义
 *   圆: x² + y² = R²
 *
 *   令 dx=x2-x1, dy=y2-y1, dr²=dx²+dy², D=x1*y2-x2*y1
 *   则交点 x = (D*dy ± sgn(dd)*dx*sqrt(R²*dr²-D²)) / dr²
 *          y = (-D*dx ± sgn(dd)*dy*sqrt(R²*dr²-D²)) / dr²
 *
 *   其中 ± 号的选取由 dd = d2-d1 = (x2²+y2²)-(x1²+y1²) 的符号决定，
 *   确保返回的是线段 P1-P2 上（而非延长线上）的交点。
 *
 * 参考: https://mathworld.wolfram.com/Circle-LineIntersection.html
 *
 * @param p1 线段起点（在圆内）
 * @param p2 线段终点（在圆外）
 * @param r  圆的半径（前视距离）
 * @return   线段上距离原点恰好为 r 的点
 */
geometry_msgs::msg::Point OmniPidPursuitController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r)
{
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double d = x1 * y2 - x2 * y1;

  // d1 = |P1|², d2 = |P2|², dd = d2 - d1
  // dd 的符号用来确定 ± 号，确保交点在 P1-P2 之间
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::msg::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - d * d);
  // copysign(1.0, dd): 取 dd 的符号，用于选择正确的交点
  p.x = (d * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-d * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

/**
 * @brief 获取代价地图的最大半边长
 *
 * 取 X 和 Y 两个方向上尺寸较大者的一半。
 * 这个值决定了路径变换时需要考虑的最大范围。
 *
 * @return 代价地图最大半边长（米）
 */
double OmniPidPursuitController::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters =
    std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

/**
 * @brief 坐标变换：将位姿从一个坐标系变换到另一个
 *
 * 使用 TF2 缓冲区进行变换。如果源帧和目标帧已相同则直接返回。
 * 变换失败时打印错误日志并返回 false。
 *
 * @param frame     目标坐标系 ID（如 "base_link", "map"）
 * @param in_pose   输入位姿（需包含正确的 frame_id）
 * @param out_pose  输出位姿（变换后的结果）
 * @return          变换成功返回 true，失败返回 false
 */
bool OmniPidPursuitController::transformPose(
  const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

/**
 * @brief 碰撞检测：检查路径是否穿越障碍物
 *
 * 遍历路径上的每个位姿点，将其世界坐标转换为代价地图的网格坐标，
 * 然后查取代价值。如果代价值 >= INSCRIBED_INFLATED_OBSTACLE
 * （内切膨胀障碍物，即机器人轮廓一定会碰撞到的区域），则认为检测到碰撞。
 *
 * 注意：如果路径点无法映射到代价地图内（worldToMap 返回 false），
 * 当前只返回 false 允许继续，不阻断运动（保守策略）。
 *
 * @param path 代价地图坐标系下的路径
 * @return     检测到碰撞返回 true，否则返回 false
 */
bool OmniPidPursuitController::isCollisionDetected(const nav_msgs::msg::Path & path)
{
  auto costmap = costmap_ros_->getCostmap();
  for (const auto & pose_stamped : path.poses) {
    const auto & pose = pose_stamped.pose;
    unsigned int mx, my;
    // 将世界坐标 (pose.x, pose.y) 转换为代价地图的网格索引 (mx, my)
    if (costmap->worldToMap(pose.position.x, pose.position.y, mx, my)) {
      // Unknown cells (NO_INFORMATION) are expected at exploration frontiers
      // and must not be treated as physical obstacles. Navfn is configured to
      // allow unknown space; the local costmap/collision monitor still handles
      // newly observed obstacles while the robot approaches it.
      const auto cost = costmap->getCost(mx, my);
      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        return true;
      }
    } else {
      // 路径点超出代价地图范围：虽然不足以判断碰撞，但应谨慎
      return false;
    }
  }
  return false;
}

/**
 * @brief 根据当前速度计算前视距离
 *
 * 两种模式：
 *   1. 固定距离模式（use_velocity_scaled_lookahead_dist = false）：
 *      直接使用 lookahead_dist_ 参数，始终保持固定前视距离。
 *   2. 速度缩放模式（use_velocity_scaled_lookahead_dist = true，默认）：
 *      前视距离 = 当前速度 × 前视时间，然后钳位到 [min, max] 范围。
 *      速度越快看得越远，速度越慢看得越近，实现自适应纯追踪。
 *      这个设计的物理直觉是：速度快时需要更早"看"到前方的弯道来提前减速。
 *
 * @param speed 机器人当前速度（Twist 消息，使用 linear.x 和 linear.y 的合速度）
 * @return      前视距离（米）
 */
double OmniPidPursuitController::getLookAheadDistance(const geometry_msgs::msg::Twist & speed)
{
  double lookahead_dist = lookahead_dist_;

  if (use_velocity_scaled_lookahead_dist_) {
    // 计算当前合速度：sqrt(vx² + vy²)，全向轮可能有侧向速度分量
    lookahead_dist = hypot(speed.linear.x, speed.linear.y) * lookahead_time_;
    // 钳位：不能小于 min，不能大于 max
    lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }

  return lookahead_dist;
}

/**
 * @brief 计算接近目标时的速度缩放因子
 *
 * 当剩余路径长度小于设定的接近减速距离时，返回一个 (0, 1] 的比例因子，
 * 使机器人在接近终点时线速度线性递减。
 *
 * 设计要点：
 *   - 使用 calculate_path_length（积分弧长）而非直线距离判断是否触发减速，
 *     避免在弯曲路径上因为末端点看起来很近而过早减速。
 *   - 触发后使用机器人原点到路径末端点的直线距离做平滑缩放，
 *     保证缩放过程连续无跳变。
 *
 * @param transformed_path 变换后的局部路径
 * @return                 速度缩放因子，范围 (0, 1]
 */
double OmniPidPursuitController::approachVelocityScalingFactor(
  const nav_msgs::msg::Path & transformed_path) const
{
  // 使用路径积分长度来判断：只有当沿路径走到终点的总距离小于减速距离时才触发
  double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_path);
  if (remaining_distance < approach_velocity_scaling_dist_) {
    auto & last = transformed_path.poses.back();
    // 使用机器人原点到最后一个路径点的直线距离做平滑缩放
    double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
    return distance_to_last_pose / approach_velocity_scaling_dist_;
  } else {
    return 1.0;
  }
}

/**
 * @brief 施加接近目标的速度缩放
 *
 * 确保最终线速度不低于 min_approach_linear_velocity（防止完全停止不动），
 * 同时取接近减速、曲率减速等多种约束中较严格的最低速度。
 *
 * @param path       局部路径
 * @param linear_vel 线速度（输入输出参数，会被原地修改）
 */
void OmniPidPursuitController::applyApproachVelocityScaling(
  const nav_msgs::msg::Path & path, double & linear_vel) const
{
  double approach_vel = linear_vel;
  double velocity_scaling = approachVelocityScalingFactor(path);
  double unbounded_vel = approach_vel * velocity_scaling;
  // 如果缩放后的速度低于最小接近速度，使用最小接近速度
  // 避免机器人在目标附近完全停止
  if (unbounded_vel < min_approach_linear_velocity_) {
    approach_vel = min_approach_linear_velocity_;
  } else {
    approach_vel *= velocity_scaling;
  }

  // 综合多种减速约束，取最严格（最低）的速度
  linear_vel = std::min(linear_vel, approach_vel);
}

/**
 * @brief 基于路径曲率的自适应性速度限制
 *
 * 核心思路：
 *   1. 在前视点附近取三个点（前点、前视点、后点）计算路径曲率
 *   2. 根据曲率大小决定降速比例：
 *      - 曲率 ≤ curvature_min（接近直线）：不降速，reduction_ratio = 1.0
 *      - curvature_min < 曲率 < curvature_max：线性插值降速
 *      - 曲率 ≥ curvature_max（急转弯）：降至 reduction_ratio_at_high_curvature
 *   3. 降速过程受 max_velocity_scaling_factor_rate 约束，
 *      速度缩放因子每轮变化量有限，保证平滑过渡，避免速度突变
 *   4. 最终缩放速度不低于 2.0 * min_approach_linear_velocity
 *
 * @param path           局部路径
 * @param lookahead_pose 前视点位姿
 * @param linear_vel     线速度（输入输出参数）
 */
void OmniPidPursuitController::applyCurvatureLimitation(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double & linear_vel)
{
  // 计算前视点附近的曲率
  double curvature =
    calculateCurvature(path, lookahead_pose, curvature_forward_dist_, curvature_backward_dist_);

  double scaled_linear_vel = linear_vel;
  if (curvature > curvature_min_) {
    double reduction_ratio = 1.0;
    if (curvature > curvature_max_) {
      // 曲率过大（急转弯）：使用预定义的大幅降速比例
      reduction_ratio = reduction_ratio_at_high_curvature_;
    } else {
      // 曲率在中间范围：线性插值计算降速比例
      // 从 1.0 线性递减到 reduction_ratio_at_high_curvature_
      reduction_ratio = 1.0 - (curvature - curvature_min_) / (curvature_max_ - curvature_min_) *
                                (1.0 - reduction_ratio_at_high_curvature_);
    }

    // 目标缩放后速度 = 当前速度 × 降速比例
    double target_scaled_vel = linear_vel * reduction_ratio;
    // 平滑过渡：限制每轮速度缩放因子的变化量，防止速度突变
    scaled_linear_vel =
      last_velocity_scaling_factor_ + std::clamp(
                                        target_scaled_vel - last_velocity_scaling_factor_,
                                        -max_velocity_scaling_factor_rate_ * control_duration_,
                                        max_velocity_scaling_factor_rate_ * control_duration_);
  }
  // 确保速度不低于最小接近速度的两倍（给出足够的运动余量）
  scaled_linear_vel = std::max(scaled_linear_vel, 2.0 * min_approach_linear_velocity_);

  // 综合约束：取所有限速中最小的那个
  linear_vel = std::min(linear_vel, scaled_linear_vel);
  applyCurvatureLookaheadLimitation(path, linear_vel);
  // 记录最终限速结果，供下一轮局部曲率限速平滑过渡使用。
  last_velocity_scaling_factor_ = linear_vel;
}

nav_msgs::msg::Path OmniPidPursuitController::smoothPath(const nav_msgs::msg::Path & path) const
{
  if (!use_path_smoothing_ || path.poses.size() < 3 || path_smoothing_iterations_ <= 0) {
    return path;
  }

  nav_msgs::msg::Path smoothed = path;
  // A conservative 1-2-1 filter reduces planner stair-steps. Endpoints stay
  // fixed and every interior point is capped to a small offset from the raw
  // path, preserving route topology for holonomic pursuit.
  for (int iteration = 0; iteration < path_smoothing_iterations_; ++iteration) {
    const auto previous = smoothed;
    for (size_t index = 1; index + 1 < smoothed.poses.size(); ++index) {
      const auto & raw = path.poses[index].pose.position;
      const auto & before = previous.poses[index - 1].pose.position;
      const auto & current = previous.poses[index].pose.position;
      const auto & after = previous.poses[index + 1].pose.position;
      const double candidate_x = 0.25 * before.x + 0.5 * current.x + 0.25 * after.x;
      const double candidate_y = 0.25 * before.y + 0.5 * current.y + 0.25 * after.y;
      const double offset_x = candidate_x - raw.x;
      const double offset_y = candidate_y - raw.y;
      const double offset = std::hypot(offset_x, offset_y);
      const double scale = offset > path_smoothing_max_offset_ && offset > 1e-9 ?
        path_smoothing_max_offset_ / offset : 1.0;
      smoothed.poses[index].pose.position.x = raw.x + offset_x * scale;
      smoothed.poses[index].pose.position.y = raw.y + offset_y * scale;
    }
  }
  return smoothed;
}

void OmniPidPursuitController::applyCurvatureLookaheadLimitation(
  const nav_msgs::msg::Path & path, double & linear_vel)
{
  if (path.poses.size() < 3 || curvature_lookahead_dist_ <= 0.0 ||
    curvature_sample_dist_ <= 0.0 || max_lateral_accel_ <= 0.0 ||
    curvature_max_deceleration_ <= 0.0)
  {
    return;
  }

  const auto cumulative = calculateCumulativeDistances(path);
  const double total_distance = cumulative.back();
  double peak_curvature = 0.0;
  double distance_to_peak = total_distance;
  const double sample_half_width = std::max(curvature_sample_dist_, 0.05);
  const double horizon = std::min(curvature_lookahead_dist_, total_distance);

  // Start one half-width from the robot so each curvature estimate has a
  // meaningful point on both sides. Sampling by arc length makes this robust
  // to planners with irregular waypoint spacing.
  for (double distance = sample_half_width; distance < horizon; distance += curvature_sample_dist_) {
    const auto near_pose = findPoseAtDistance(path, cumulative, distance - sample_half_width);
    const auto center_pose = findPoseAtDistance(path, cumulative, distance);
    const auto far_pose = findPoseAtDistance(path, cumulative, distance + sample_half_width);
    const double radius = calculateCurvatureRadius(
      near_pose.pose.position, center_pose.pose.position, far_pose.pose.position);
    const double curvature = 1.0 / radius;
    if (std::isfinite(curvature) && curvature > peak_curvature) {
      peak_curvature = curvature;
      distance_to_peak = distance;
    }
  }
  if (peak_curvature <= curvature_min_) {
    return;
  }

  // v^2 * curvature <= a_lateral gives a physics-based curve speed. The
  // braking bound ensures that speed can be reduced before that curve begins.
  const double curve_speed = std::sqrt(max_lateral_accel_ / peak_curvature);
  const double braking_speed = std::sqrt(std::max(
    0.0, curve_speed * curve_speed + 2.0 * curvature_max_deceleration_ * distance_to_peak));
  linear_vel = std::min(linear_vel, braking_speed);
}

/**
 * @brief 在路径上取三点计算曲率
 *
 * 步骤：
 *   1. 计算路径每个点到起点的累积距离
 *   2. 找到前视点在路径上的累积距离位置
 *   3. 在前视点前后的累积距离轴上各偏移一定距离，找到"前点"和"后点"
 *   4. 三点求曲率半径，取倒数得曲率
 *   5. 发布前后点的可视化标记（用于调试）
 *
 * 前视点在路径上的累积距离近似为其与机器人原点的欧几里得距离，
 * 因为在 base_link 系下机器人位于原点。
 *
 * @param path           局部路径
 * @param lookahead_pose 前视点位姿
 * @param forward_dist   向前搜索距离
 * @param backward_dist  向后搜索距离
 * @return               曲率 = 1/曲率半径
 */
double OmniPidPursuitController::calculateCurvature(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double forward_dist, double backward_dist) const
{
  geometry_msgs::msg::PoseStamped backward_pose, forward_pose;
  std::vector<double> cumulative_distances = calculateCumulativeDistances(path);

  // 前视点的累积距离 ≈ 前视点到原点的欧几里得距离
  double lookahead_pose_cumulative_distance = 0.0;
  geometry_msgs::msg::PoseStamped robot_base_frame_pose;
  robot_base_frame_pose.pose = geometry_msgs::msg::Pose();
  lookahead_pose_cumulative_distance =
    nav2_util::geometry_utils::euclidean_distance(robot_base_frame_pose, lookahead_pose);

  // 在路径上找到前视点后方 backward_dist 处的点
  backward_pose = findPoseAtDistance(
    path, cumulative_distances, lookahead_pose_cumulative_distance - backward_dist);

  // 在路径上找到前视点前方 forward_dist 处的点
  forward_pose = findPoseAtDistance(
    path, cumulative_distances, lookahead_pose_cumulative_distance + forward_dist);

  // 三点计算曲率半径（后点、前视点、前点）
  double curvature_radius = calculateCurvatureRadius(
    backward_pose.pose.position, lookahead_pose.pose.position, forward_pose.pose.position);
  double curvature = 1.0 / curvature_radius;
  visualizeCurvaturePoints(backward_pose, forward_pose);
  return curvature;
}

/**
 * @brief 使用三点几何方法计算曲率半径
 *
 * 给定三个不共线的点，存在唯一的过这三点的圆（外接圆）。
 * 曲率 = 1/曲率半径，曲率越大表示转弯越急。
 *
 * 计算方法：
 *   圆的一般方程：(x - cx)² + (y - cy)² = R²
 *   圆心 (cx, cy) 是两条垂直平分线的交点，公式通过解析几何推导：
 *
 *   cx = [ (x1²+y1²)(y2-y3) + (x2²+y2²)(y3-y1) + (x3²+y3²)(y1-y2) ] / denominator
 *   cy = [ (x1²+y1²)(x3-x2) + (x2²+y2²)(x1-x3) + (x3²+y3²)(x2-x1) ] / denominator
 *   denominator = 2 * [ x1(y2-y3) + x2(y3-y1) + x3(y1-y2) ]
 *
 *   R = sqrt( (x2-cx)² + (y2-cy)² )
 *
 * 边界保护：
 *   - 如果半径为 NaN 或无穷大（三点共线），返回 1e9（近似直线）
 *   - 如果半径 < 1e-9（数值不稳定），返回 1e9
 *
 * @param near_point    后点（路径上靠后的点）
 * @param current_point 当前点（前视点）
 * @param far_point     前点（路径上靠前的点）
 * @return              曲率半径（米）
 */
double OmniPidPursuitController::calculateCurvatureRadius(
  const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
  const geometry_msgs::msg::Point & far_point) const
{
  double x1 = near_point.x, y1 = near_point.y;
  double x2 = current_point.x, y2 = current_point.y;
  double x3 = far_point.x, y3 = far_point.y;

  // 分母 = 2 * 三点面积（带符号）× 2
  double center_x = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) +
                     (x3 * x3 + y3 * y3) * (y1 - y2)) /
                    (2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)));
  double center_y = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) +
                     (x3 * x3 + y3 * y3) * (x2 - x1)) /
                    (2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)));
  double radius = std::hypot(x2 - center_x, y2 - center_y);

  // 数值保护：如果半径无效（三点共线或数值不稳定），返回一个非常大的值
  // 1e9 对应的曲率 = 1e-9，几乎为零，等同于直线
  if (std::isnan(radius) || std::isinf(radius) || radius < 1e-9) {
    return 1e9;
  }
  return radius;
}

/**
 * @brief 发布曲率计算所用前后点的可视化标记
 *
 * 在 RViz 中创建两个球形 Marker：
 *   - 绿色球（id=0）：后点 (backward_pose)，显示曲率计算的后参考点
 *   - 红色球（id=1）：前点 (forward_pose)，显示曲率计算的前参考点
 *
 * 用于调试曲率计算是否正确选取了前后参考点。
 *
 * @param backward_pose 后参考点位姿
 * @param forward_pose  前参考点位姿
 */
void OmniPidPursuitController::visualizeCurvaturePoints(
  const geometry_msgs::msg::PoseStamped & backward_pose,
  const geometry_msgs::msg::PoseStamped & forward_pose) const
{
  visualization_msgs::msg::MarkerArray marker_array;

  // 后点标记：绿色球
  visualization_msgs::msg::Marker near_marker;
  near_marker.header = backward_pose.header;
  near_marker.ns = "curvature_points";
  near_marker.id = 0;
  near_marker.type = visualization_msgs::msg::Marker::SPHERE;
  near_marker.action = visualization_msgs::msg::Marker::ADD;
  near_marker.pose = backward_pose.pose;
  near_marker.scale.x = near_marker.scale.y = near_marker.scale.z = 0.1;
  near_marker.color.g = 1.0;  // 绿色
  near_marker.color.a = 1.0;  // 完全不透明

  // 前点标记：红色球
  visualization_msgs::msg::Marker far_marker;
  far_marker.header = forward_pose.header;
  far_marker.ns = "curvature_points";
  far_marker.id = 1;
  far_marker.type = visualization_msgs::msg::Marker::SPHERE;
  far_marker.action = visualization_msgs::msg::Marker::ADD;
  far_marker.pose = forward_pose.pose;
  far_marker.scale.x = far_marker.scale.y = far_marker.scale.z = 0.1;
  far_marker.color.r = 1.0;  // 红色
  far_marker.color.a = 1.0;

  marker_array.markers.push_back(near_marker);
  marker_array.markers.push_back(far_marker);

  curvature_points_pub_->publish(marker_array);
}

/**
 * @brief 计算路径上每点到起点的累积弧长距离
 *
 * 返回的向量中 cumulative_distances[i] 是从路径第 0 个点
 * 沿路径走到第 i 个点的总距离（各段欧几里得距离之和）。
 * 第一个元素始终为 0。
 *
 * 这个累积距离数组在 findPoseAtDistance 中用于二分查找
 * 某个特定距离处的位姿。
 *
 * @param path 输入路径
 * @return     累积距离向量，大小与 path.poses 相同
 */
std::vector<double> OmniPidPursuitController::calculateCumulativeDistances(
  const nav_msgs::msg::Path & path) const
{
  std::vector<double> cumulative_distances;
  cumulative_distances.push_back(0.0);

  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & prev_pose = path.poses[i - 1].pose.position;
    const auto & curr_pose = path.poses[i].pose.position;
    // 计算相邻两点之间的欧几里得距离并累加
    double distance = hypot(curr_pose.x - prev_pose.x, curr_pose.y - prev_pose.y);
    cumulative_distances.push_back(cumulative_distances.back() + distance);
  }
  return cumulative_distances;
}

/**
 * @brief 在路径上找到恰好距离起点 target_distance 处的位姿
 *
 * 实现方式：
 *   1. 通过 std::lower_bound 在累积距离数组中找到 target_distance 所在区间
 *   2. 在线段的两个端点之间做线性插值
 *
 * 边界情况：
 *   - 路径为空：返回空位姿
 *   - target_distance ≤ 0：返回路径第一个点
 *   - target_distance ≥ 路径总长：返回路径最后一个点
 *
 * 插值公式：
 *   ratio = (target - dist[i-1]) / (dist[i] - dist[i-1])
 *   result = pose[i-1] + ratio * (pose[i] - pose[i-1])
 *
 * @param path                 输入路径
 * @param cumulative_distances 累积距离数组
 * @param target_distance      目标弧长距离
 * @return                     插值后的精确位姿
 */
geometry_msgs::msg::PoseStamped OmniPidPursuitController::findPoseAtDistance(
  const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
  double target_distance) const
{
  if (path.poses.empty() || cumulative_distances.empty()) {
    return geometry_msgs::msg::PoseStamped();
  }
  if (target_distance <= 0.0) {
    return path.poses.front();
  }
  if (target_distance >= cumulative_distances.back()) {
    return path.poses.back();
  }

  // 二分查找：找到第一个累积距离 >= target_distance 的位置
  auto it =
    std::lower_bound(cumulative_distances.begin(), cumulative_distances.end(), target_distance);
  size_t index = std::distance(cumulative_distances.begin(), it);

  if (index == 0) {
    return path.poses.front();
  }

  // 在 cumulative_distances[index-1] 和 cumulative_distances[index] 之间线性插值
  // ratio = 0 -> 落在 poses[index-1], ratio = 1 -> 落在 poses[index]
  double ratio = (target_distance - cumulative_distances[index - 1]) /
                 (cumulative_distances[index] - cumulative_distances[index - 1]);
  geometry_msgs::msg::PoseStamped pose1 = path.poses[index - 1];
  geometry_msgs::msg::PoseStamped pose2 = path.poses[index];

  geometry_msgs::msg::PoseStamped interpolated_pose;
  interpolated_pose.header = pose2.header;
  // 线性插值：位置 (X, Y, Z)
  interpolated_pose.pose.position.x =
    pose1.pose.position.x + ratio * (pose2.pose.position.x - pose1.pose.position.x);
  interpolated_pose.pose.position.y =
    pose1.pose.position.y + ratio * (pose2.pose.position.y - pose1.pose.position.y);
  interpolated_pose.pose.position.z =
    pose1.pose.position.z + ratio * (pose2.pose.position.z - pose1.pose.position.z);
  // 姿态不插值，直接使用目标区间的末端姿态
  interpolated_pose.pose.orientation = pose2.pose.orientation;

  return interpolated_pose;
}

/**
 * @brief 动态参数更新回调
 *
 * 当用户通过 ros2 param set 或 rqt_reconfigure 修改参数时，
 * Nav2 的 parameter_handler 会调用此函数。
 *
 * 在互斥锁保护下安全地更新成员变量，保证参数更新不会与
 * computeVelocityCommands 中的速度计算发生竞态条件。
 *
 * 支持的参数：
 *   double 类型：所有 PID 增益、速度限制、前视距离、曲率参数
 *   bool 类型：use_velocity_scaled_lookahead_dist、use_interpolation、use_rotate_to_heading
 *
 * @param parameters 发生变化的参数列表
 * @return           全部处理成功则 success=true
 */
rcl_interfaces::msg::SetParametersResult OmniPidPursuitController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  double next_min_lookahead = min_lookahead_dist_;
  double next_max_lookahead = max_lookahead_dist_;
  double next_curvature_min = curvature_min_;
  double next_curvature_max = curvature_max_;
  double next_integral_limit = min_max_sum_error_;
  double next_smoothing_offset = path_smoothing_max_offset_;
  double next_lookahead_horizon = curvature_lookahead_dist_;
  double next_sample_dist = curvature_sample_dist_;
  double next_lateral_accel = max_lateral_accel_;
  double next_deceleration = curvature_max_deceleration_;
  int next_smoothing_iterations = path_smoothing_iterations_;
  for (const auto & parameter : parameters) {
    if (parameter.get_type() != ParameterType::PARAMETER_DOUBLE) {
      continue;
    }
    const double value = parameter.as_double();
    if (!std::isfinite(value)) {
      result.successful = false;
      result.reason = "Controller parameters must be finite";
      return result;
    }
    const auto & name = parameter.get_name();
    if (name == plugin_name_ + ".min_lookahead_dist") next_min_lookahead = value;
    else if (name == plugin_name_ + ".max_lookahead_dist") next_max_lookahead = value;
    else if (name == plugin_name_ + ".curvature_min") next_curvature_min = value;
    else if (name == plugin_name_ + ".curvature_max") next_curvature_max = value;
    else if (name == plugin_name_ + ".min_max_sum_error") next_integral_limit = value;
    else if (name == plugin_name_ + ".path_smoothing_max_offset") next_smoothing_offset = value;
    else if (name == plugin_name_ + ".curvature_lookahead_dist") next_lookahead_horizon = value;
    else if (name == plugin_name_ + ".curvature_sample_dist") next_sample_dist = value;
    else if (name == plugin_name_ + ".max_lateral_accel") next_lateral_accel = value;
    else if (name == plugin_name_ + ".curvature_max_deceleration") next_deceleration = value;
  }
  for (const auto & parameter : parameters) {
    if (parameter.get_type() == ParameterType::PARAMETER_INTEGER &&
      parameter.get_name() == plugin_name_ + ".path_smoothing_iterations")
    {
      next_smoothing_iterations = static_cast<int>(parameter.as_int());
    }
  }
  if (next_min_lookahead <= 0.0 || next_max_lookahead < next_min_lookahead ||
    next_curvature_min < 0.0 || next_curvature_max <= next_curvature_min ||
    next_integral_limit < 0.0 || next_smoothing_offset < 0.0 ||
    next_lookahead_horizon < 0.0 || next_sample_dist <= 0.0 ||
    next_lateral_accel <= 0.0 || next_deceleration <= 0.0 || next_smoothing_iterations < 0)
  {
    result.successful = false;
    result.reason = "Invalid lookahead, curvature, or integral-limit parameter relation";
    return result;
  }

  for (const auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".translation_kp") {
        translation_kp_ = parameter.as_double();
      } else if (name == plugin_name_ + ".translation_ki") {
        translation_ki_ = parameter.as_double();
      } else if (name == plugin_name_ + ".translation_kd") {
        translation_kd_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_kp") {
        rotation_kp_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_ki") {
        rotation_ki_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_kd") {
        rotation_kd_ = parameter.as_double();
      } else if (name == plugin_name_ + ".transform_tolerance") {
        double transform_tolerance = parameter.as_double();
        transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
      } else if (name == plugin_name_ + ".min_max_sum_error") {
        min_max_sum_error_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_dist") {
        lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_lookahead_dist") {
        min_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_lookahead_dist") {
        max_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_time") {
        lookahead_time_ = parameter.as_double();
      } else if (name == plugin_name_ + ".use_rotate_to_heading_treshold") {
        use_rotate_to_heading_treshold_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_approach_linear_velocity") {
        min_approach_linear_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".approach_velocity_scaling_dist") {
        approach_velocity_scaling_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_max") {
        v_linear_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_min") {
        v_linear_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_angular_max") {
        v_angular_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_angular_min") {
        v_angular_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_min") {
        curvature_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_max") {
        curvature_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".reduction_ratio_at_high_curvature") {
        reduction_ratio_at_high_curvature_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_forward_dist") {
        curvature_forward_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_backward_dist") {
        curvature_backward_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_velocity_scaling_factor_rate") {
        max_velocity_scaling_factor_rate_ = parameter.as_double();
      } else if (name == plugin_name_ + ".path_smoothing_max_offset") {
        path_smoothing_max_offset_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_lookahead_dist") {
        curvature_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_sample_dist") {
        curvature_sample_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_lateral_accel") {
        max_lateral_accel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_max_deceleration") {
        curvature_max_deceleration_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == plugin_name_ + ".path_smoothing_iterations") {
        path_smoothing_iterations_ = static_cast<int>(parameter.as_int());
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".use_velocity_scaled_lookahead_dist") {
        use_velocity_scaled_lookahead_dist_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_interpolation") {
        use_interpolation_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_rotate_to_heading") {
        use_rotate_to_heading_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_path_smoothing") {
        use_path_smoothing_ = parameter.as_bool();
      }
    }
  }

  if (move_pid_) {
    move_pid_->setGains(translation_kp_, translation_kd_, translation_ki_);
    move_pid_->setLimits(v_linear_max_, v_linear_min_);
    move_pid_->setIntegralLimit(min_max_sum_error_);
  }
  if (heading_pid_) {
    heading_pid_->setGains(rotation_kp_, rotation_kd_, rotation_ki_);
    heading_pid_->setLimits(v_angular_max_, v_angular_min_);
    heading_pid_->setIntegralLimit(min_max_sum_error_);
  }

  result.successful = true;
  return result;
}

};  // namespace pb_omni_pid_pursuit_controller

// ========== 插件注册 ==========
// 将此控制器类注册为 nav2_core::Controller 插件
// 使 Nav2 的 pluginlib 能够动态加载本控制器
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pb_omni_pid_pursuit_controller::OmniPidPursuitController, nav2_core::Controller)
