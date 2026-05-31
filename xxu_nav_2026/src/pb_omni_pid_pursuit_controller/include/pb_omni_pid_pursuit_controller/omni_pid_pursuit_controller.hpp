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

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_

#include <mutex>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pb_omni_pid_pursuit_controller/pid.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace pb_omni_pid_pursuit_controller
{

/**
 * @class pb_omni_pid_pursuit_controller::OmniPidPursuitController
 * @brief 全向轮 PID 纯追踪控制器 —— Nav2 控制器插件
 *
 * 该控制器基于纯追踪算法（Pure Pursuit），针对全向移动机器人做了扩展。
 * 与传统差速机器人的纯追踪不同，本控制器同时控制 X、Y 方向的线速度
 * 和 Z 轴的角速度，充分发挥全向轮底盘的 omnidirectional 运动能力。
 *
 * 核心设计思路：
 *   1. 纯追踪前视点：在全局路径上找到距离机器人一定前视距离（lookahead）
 *      的目标点，该点即"胡萝卜点"（carrot point）。
 *   2. 双 PID 控制：一个 PID 控制器负责追踪到前视点的直线距离（平移），
 *      另一个 PID 控制器负责追踪目标朝向（旋转）。两个 PID
 *      独立运行，解耦控制。
 *   3. 曲率自适应降速：根据路径曲率动态降低线速度，在急转弯处减速。
 *   4. 接近目标减速：当接近全局路径终点时，线性缩放速度以平滑停止。
 *   5. 碰撞检测：在代价地图中检查路径是否穿越障碍物。
 *
 * 典型使用场景：
 *   - 全向轮/麦克纳姆轮底盘的导航控制
 *   - 需要解耦平移和旋转控制的场景
 *   - 需要曲率自适应调速的复杂路径跟踪
 */
class OmniPidPursuitController : public nav2_core::Controller
{
public:
  OmniPidPursuitController() = default;

  ~OmniPidPursuitController() override = default;

  /**
   * @brief 配置控制器 —— 生命周期配置阶段
   *
   * 完成以下初始化工作：
   *   - 获取节点指针、TF 缓冲区、代价地图 ROS 包装器
   *   - 声明并读取所有控制参数（PID 增益、前视距离、速度限制、曲率参数等）
   *   - 创建两个 PID 控制器（平移 + 旋转）
   *   - 创建用于调试可视化的发布者（局部路径、胡萝卜点、曲率点）
   *
   * @param parent     生命周期节点的弱引用指针
   * @param name       插件名称，用于参数命名空间隔离
   * @param tf         TF2 坐标变换缓冲区
   * @param costmap_ros 代价地图 ROS2 节点包装器
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief 清理控制器资源 —— 生命周期清理阶段
   *
   * 重置所有发布者的共享指针，释放资源。
   */
  void cleanup() override;

  /**
   * @brief 激活控制器 —— 生命周期激活阶段
   *
   * 激活所有发布者并注册动态参数回调，控制器开始接受 computeVelocityCommands 调用。
   */
  void activate() override;

  /**
   * @brief 停用控制器 —— 生命周期停用阶段
   *
   * 停用所有发布者并移除动态参数回调，控制器停止计算速度指令。
   */
  void deactivate() override;

  /**
   * @brief 计算最佳速度指令 —— 控制器核心函数
   *
   * 每轮控制循环被 Nav2 调用一次，根据当前机器人位姿和速度计算输出指令。
   *
   * 执行流程：
   *   1. 将全局路径变换到机器人坐标系
   *   2. 根据当前速度计算前视距离
   *   3. 在路径上找到前视点（胡萝卜点）
   *   4. 用平移 PID 计算距离到前视点的线速度
   *   5. 用旋转 PID 计算朝向误差的角速度
   *   6. 施加曲率限速和接近减速
   *   7. 碰撞检测，若检测到碰撞则抛出异常使行为树进入恢复状态
   *
   * @param pose         机器人当前在全局坐标系中的位姿
   * @param velocity     机器人当前速度 (Twist 消息)
   * @param goal_checker 目标检查器指针（本控制器未使用，保留接口兼容）
   * @return             带时间戳的速度指令 (TwistStamped)
   * @throws nav2_core::NoValidControl 当检测到碰撞时抛出
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief 设置全局规划路径
   *
   * 由 Nav2 规划器在产生新路径时调用，路径会被存储并在后续的
   * computeVelocityCommands 中被逐步裁剪（路径剪枝）。
   *
   * @param path 规划器生成的全局导航路径
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief 设置速度限制（本控制器未实现）
   *
   * @param speed_limit 速度限制值（绝对值或百分比）
   * @param percentage  true 表示百分比，false 表示绝对值
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief 将全局路径变换到机器人基座坐标系并进行路径剪枝
   *
   * 处理步骤：
   *   1. 将机器人位姿变换到全局路径的坐标系
   *   2. 在全局路径上找到离机器人最近的位姿点
   *   3. 将 [最近点, 代价地图边界] 范围内的位姿变换到机器人基座系
   *   4. 从全局路径中移除已走过的部分（路径剪枝，避免重复处理）
   *
   * @param pose 机器人当前在全局坐标系中的位姿
   * @return     在机器人基座坐标系下的变换后路径
   */
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief 将一个位姿从当前坐标系变换到目标坐标系
   *
   * 使用 TF2 缓冲区进行坐标变换，如果源坐标系与目标坐标系相同则直接返回。
   *
   * @param frame     目标坐标系名称（如 "base_link", "map"）
   * @param in_pose   输入位姿（含源坐标系信息）
   * @param out_pose  输出位姿（变换后结果）
   * @return          变换成功返回 true，失败返回 false
   */
  bool transformPose(
    const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  /**
   * @brief 获取代价地图的最大半边长（以米为单位）
   *
   * 取 X 和 Y 方向上尺寸较大者的一半作为返回值，
   * 用于确定路径变换的搜索范围。
   *
   * @return 代价地图最大半边长（米）
   */
  double getCostmapMaxExtent() const;

  /**
   * @brief 创建胡萝卜点（前视点）的可视化标记消息
   *
   * 该点位于 z=0.01 处，略高于地面以便在 RViz 中显眼显示。
   *
   * @param carrot_pose 前视点的位姿
   * @return            胡萝卜点标记消息的独占指针
   */
  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief 在变换后的局部路径上查找前视点
   *
   * 从路径起点（机器人原点）开始，沿路径方向寻找第一个距离
   * 大于前视距离的位姿点。如果启用了插值，会在圆与路径线段的
   * 交点处精确计算前视点位置。
   *
   * @param lookahead_dist    前视距离（米）
   * @param transformed_plan  已在机器人坐标系下的局部路径
   * @return                  前视点的位姿
   */
  geometry_msgs::msg::PoseStamped getLookAheadPoint(
    const double & lookahead_dist, const nav_msgs::msg::Path & transformed_plan);

  /**
   * @brief 计算圆与线段在原点为圆心的坐标系中的交点
   *
   * 使用标准圆-线交点公式的封闭解，并通过符号函数确保返回的是
   * 位于线段 p1-p2 之间的那个交点（而不是线段的延长线上）。
   *
   * 数学原理参考：
   * https://mathworld.wolfram.com/Circle-LineIntersection.html
   *
   * @param p1 线段起点（已经在机器人坐标系中）
   * @param p2 线段终点
   * @param r  前视圆的半径（即前视距离）
   * @return   线段上距离圆心恰好为 r 的交点
   */
  geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r);

  /**
   * @brief 动态参数更新回调函数
   *
   * 当用户在运行时通过 rqt_reconfigure 或命令行修改参数时被调用。
   * 在互斥锁保护下安全地更新所有控制器参数（PID 增益、前视距离、速度限制等）。
   *
   * @param parameters 发生变化的参数列表
   * @return           参数设置结果（成功/失败）
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  /**
   * @brief 根据当前速度计算前视距离
   *
   * 若启用速度缩放（use_velocity_scaled_lookahead_dist），
   * 则 lookahead_dist = current_speed * lookahead_time，
   * 结果被钳位在 [min_lookahead_dist, max_lookahead_dist]。
   * 速度越快看得越远，速度越慢看得越近，实现自适应纯追踪。
   *
   * @param speed 机器人当前速度
   * @return      计算后的前视距离（米）
   */
  double getLookAheadDistance(const geometry_msgs::msg::Twist & speed);

  /**
   * @brief 计算接近目标的减速比例因子
   *
   * 当剩余路径长度小于设定的减速距离时，返回一个 (0, 1] 的比例因子，
   * 使机器人在接近终点时平滑减速。使用积分距离而非端点距离
   * 来判断，避免在弯曲路径上过早触发减速。
   *
   * @param path  变换后的局部路径
   * @return      速度缩放因子，范围 (0, 1]
   */
  double approachVelocityScalingFactor(const nav_msgs::msg::Path & path) const;

  /**
   * @brief 根据接近目标距离施加速度缩放
   *
   * 综合考虑接近减速因子和最小接近速度，限制最终线速度。
   * 优先确保不低于 min_approach_linear_velocity，
   * 同时取接近减速和其他约束（曲率减速）中较严格的值。
   *
   * @param path       变换后的局部路径
   * @param linear_vel 线速度指令（输入输出参数，会被修改）
   */
  void applyApproachVelocityScaling(const nav_msgs::msg::Path & path, double & linear_vel) const;

  /**
   * @brief 检查路径上是否存在碰撞
   *
   * 在代价地图坐标系中对路径上的采样点逐个检查，
   * 若某个点的代价值 >= INSCRIBED_INFLATED_OBSTACLE（内切膨胀障碍物），
   * 则认为发生碰撞。
   *
   * @param path 代价地图坐标系下的局部路径
   * @return     如果检测到碰撞返回 true，否则返回 false
   */
  bool isCollisionDetected(const nav_msgs::msg::Path & path);

private:
  /**
   * @brief 基于路径曲率施加速度限制
   *
   * 在前视点附近提取三个点计算曲率半径，根据曲率大小进行降速：
   *   - 曲率 < curvature_min：不降速（曲率很小，近似直线）
   *   - curvature_min ≤ 曲率 < curvature_max：线性插值降速
   *   - 曲率 ≥ curvature_max：按 reduction_ratio_at_high_curvature 比例大幅降速
   *
   * 降速过程受 max_velocity_scaling_factor_rate 限制，保证平滑过渡。
   *
   * @param path           变换后的局部路径
   * @param lookahead_pose 前视点位姿
   * @param linear_vel     线速度指令（输入输出参数）
   */
  void applyCurvatureLimitation(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double & linear_vel);

  /**
   * @brief 用三点圆弧拟合法计算路径在前视点处的曲率
   *
   * 选取前视点前方 forward_dist 处和后方 backward_dist 处的两个点，
   * 与前视点一起构成三点，然后调用三点求曲率半径的几何方法。
   *
   * @param path           变换后的局部路径
   * @param lookahead_pose 前视点位姿（作为三点中的当前点）
   * @param forward_dist   从当前点向前搜索的距离
   * @param backward_dist  从当前点向后搜索的距离
   * @return               曲率值 = 1/曲率半径（1/米）
   */
  double calculateCurvature(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double forward_dist, double backward_dist) const;

  /**
   * @brief 通过三点计算曲率半径
   *
   * 给定三个点 (x1,y1), (x2,y2), (x3,y3)，使用解析几何公式计算
   * 过这三点的唯一圆的半径。公式通过求解两条垂直平分线的交点
   * 得到圆心坐标，再计算圆心到任意一点的距离得到半径。
   *
   * @param near_point    后点（靠近机器人的点）
   * @param current_point 当前点（前视点）
   * @param far_point     前点（远离机器人的点）
   * @return              曲率半径（米），若三点共线或接近共线则返回 1e9
   */
  double calculateCurvatureRadius(
    const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
    const geometry_msgs::msg::Point & far_point) const;

  /**
   * @brief 发布曲率计算所用的前后点可视化标记
   *
   * 在 RViz 中以球形 Marker 显示：
   *   - 绿色球（id=0）：后点 (backward_pose)
   *   - 红色球（id=1）：前点 (forward_pose)
   * 用于调试曲率计算是否选取了正确的点。
   *
   * @param backward_pose 曲率计算中的后点
   * @param forward_pose  曲率计算中的前点
   */
  void visualizeCurvaturePoints(
    const geometry_msgs::msg::PoseStamped & backward_pose,
    const geometry_msgs::msg::PoseStamped & forward_pose) const;

  /**
   * @brief 计算路径上每个点距离起点的累积距离
   *
   * 返回一个向量，其中第 i 个元素是从路径第一个点沿路径走到
   * 第 i 个点的总距离（非直线距离，是沿路径的弧长）。
   * 第一个元素总是 0。
   *
   * @param path  输入路径
   * @return      累积距离向量
   */
  std::vector<double> calculateCumulativeDistances(const nav_msgs::msg::Path & path) const;

  /**
   * @brief 在路径上找到距离起点恰好为 target_distance 的位姿
   *
   * 使用二分查找在累积距离数组中定位目标距离所在区间，
   * 然后在线段端点间线性插值得到精确位姿。
   *
   * 边界情况处理：
   *   - target_distance ≤ 0：返回路径第一个点
   *   - target_distance ≥ 路径总长：返回路径最后一个点
   *
   * @param path                 输入路径
   * @param cumulative_distances 累积距离向量（由 calculateCumulativeDistances 计算）
   * @param target_distance      目标距离
   * @return                     插值后的位姿
   */
  geometry_msgs::msg::PoseStamped findPoseAtDistance(
    const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
    double target_distance) const;

private:
  // === Node & TF ===
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;  ///< 生命周期节点弱引用
  std::shared_ptr<tf2_ros::Buffer> tf_;             ///< TF2 坐标变换缓冲区
  std::string plugin_name_;                          ///< 插件名称（用于参数命名空间）
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;  ///< 代价地图 ROS 包装器
  nav2_costmap_2d::Costmap2D * costmap_;             ///< 代价地图原始指针（高频访问用）
  rclcpp::Logger logger_{rclcpp::get_logger("OmniPidPursuitController")};  ///< ROS2 日志器
  rclcpp::Clock::SharedPtr clock_;                   ///< ROS2 时钟共享指针
  double last_velocity_scaling_factor_{0.0};         ///< 上一轮的速度缩放因子（用于平滑过渡）

  // === PID 控制器 ===
  std::shared_ptr<PID> move_pid_;     ///< 平移 PID 控制器（控制 X/Y 方向的线速度）
  std::shared_ptr<PID> heading_pid_;  ///< 旋转 PID 控制器（控制 Z 轴的角速度）

  // === 控制器参数 ===
  double translation_kp_, translation_ki_, translation_kd_;  ///< 平移 PID 增益 (P/I/D)
  bool enable_rotation_;                                      ///< 是否启用旋转控制
  double rotation_kp_, rotation_ki_, rotation_kd_;            ///< 旋转 PID 增益 (P/I/D)
  double min_max_sum_error_;                                  ///< 最大/最小累积误差限制
  double control_duration_;                                   ///< 控制周期（秒）= 1/控制频率
  double max_robot_pose_search_dist_;                         ///< 路径上搜索机器人最近点的最大距离
  bool use_interpolation_;                                    ///< 是否使用线段-圆插值查找前视点
  double lookahead_dist_;                                     ///< 固定前视距离（未启用速度缩放时使用）
  bool use_velocity_scaled_lookahead_dist_;                   ///< 是否根据速度动态调整前视距离
  double min_lookahead_dist_;                                 ///< 前视距离下限
  double max_lookahead_dist_;                                 ///< 前视距离上限
  double lookahead_time_;                                     ///< 前视时间（速度 × 时间 = 距离）
  bool use_rotate_to_heading_;                                ///< 是否先旋转朝向目标再前进
  double use_rotate_to_heading_treshold_;                     ///< 旋转到位角度阈值（弧度）
  double v_linear_min_;                                       ///< 最小线速度（m/s，允许负值后退）
  double v_linear_max_;                                       ///< 最大线速度（m/s）
  double v_angular_min_;                                      ///< 最小角速度（rad/s）
  double v_angular_max_;                                      ///< 最大角速度（rad/s）
  double min_approach_linear_velocity_;                       ///< 接近目标时的最小线速度
  double approach_velocity_scaling_dist_;                     ///< 接近减速的触发距离（米）
  double curvature_min_;                                      ///< 曲率下限（低于此值不减速）
  double curvature_max_;                                      ///< 曲率上限（高于此值最大减速）
  double reduction_ratio_at_high_curvature_;                  ///< 高曲率时的速度缩减比例 (0~1)
  double curvature_forward_dist_;                             ///< 曲率计算时向前搜索的距离
  double curvature_backward_dist_;                            ///< 曲率计算时向后搜索的距离
  double max_velocity_scaling_factor_rate_;                   ///< 速度缩放因子的最大变化率（防突变）
  tf2::Duration transform_tolerance_;                         ///< TF 坐标变换的容忍时间

  // === 路径数据 ===
  nav_msgs::msg::Path global_plan_;  ///< 存储的全局规划路径（会被逐步剪枝）

  // === 调试/可视化发布者 ===
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;     ///< 发布局部路径
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;  ///< 发布胡萝卜点
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    curvature_points_pub_;  ///< 发布曲率计算点可视化

  // === 动态参数 ===
  std::mutex mutex_;  ///< 互斥锁，保护参数更新和速度计算之间的并发访问
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;  ///< 动态参数回调句柄
};

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_
