// goal_approach_controller.cpp
// Nav2控制器wrapper：在接近目标时限制线速度，防止高速冲过目标点
//
// 原理：透明代理内部控制器（如MPPI），仅在距目标 < approach_distance 时
//       将合速度钳位到 approach_velocity
//
// 两种减速策略：
//   - 常规减速区 (dist < approach_distance)：保持内部控制器的方向输出，
//     仅按比例缩放线速度和角速度，使合速度不超过 approach_velocity
//   - 直接驱动区 (dist < direct_approach_distance)：完全绕过内部控制器，
//     直接朝目标点方向以 P 控制方式驱动，同时强制角速度归零，确保稳定停止

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_core/controller_exceptions.hpp"

namespace goal_approach_controller
{

// GoalApproachController：Nav2 Controller 插件，作为内部控制器的透明代理
// 只在机器人接近目标点时介入，通过速度钳位和直接驱动防止过冲
class GoalApproachController : public nav2_core::Controller
{
public:
  GoalApproachController() = default;
  ~GoalApproachController() override = default;

  // 配置插件：声明参数、加载内部控制器、获取参数值
  // 参数列表：
  //   inner_plugin               - 内部控制器类型（默认 MPPIController）
  //   approach_distance          - 开始减速的距离阈值 (m)
  //   approach_velocity          - 减速后的最大合速度 (m/s)
  //   direct_approach_distance   - 切换到直接驱动模式的距离阈值 (m)
  //   direct_approach_kp         - 直接驱动模式下的 P 增益
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto node = parent.lock();
    logger_ = node->get_logger();
    tf_ = tf;

    // 声明本wrapper的命名空间下所有参数，若无用户覆盖则使用默认值
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".inner_plugin",
      rclcpp::ParameterValue("nav2_mppi_controller::MPPIController"));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".approach_distance",
      rclcpp::ParameterValue(1.5));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".approach_velocity",
      rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".direct_approach_distance",
      rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".direct_approach_kp",
      rclcpp::ParameterValue(1.0));

    // 读取参数值到成员变量
    std::string inner_plugin_type;
    node->get_parameter(name + ".inner_plugin", inner_plugin_type);
    node->get_parameter(name + ".approach_distance", approach_distance_);
    node->get_parameter(name + ".approach_velocity", approach_velocity_);
    node->get_parameter(name + ".direct_approach_distance", direct_approach_distance_);
    node->get_parameter(name + ".direct_approach_kp", direct_approach_kp_);
    setupDynamicParameters(node, name);

    // 通过 pluginlib 动态加载内部控制器实例（支持运行时替换）
    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
      "nav2_core", "nav2_core::Controller");
    inner_controller_ = loader_->createUniqueInstance(inner_plugin_type);
    inner_controller_->configure(parent, name, tf, costmap_ros);

    RCLCPP_INFO(
      logger_,
      "GoalApproachController: 包装 [%s], approach_distance=%.2f m, approach_velocity=%.2f m/s, "
      "direct_approach_distance=%.2f m, direct_approach_kp=%.2f",
      inner_plugin_type.c_str(), approach_distance_, approach_velocity_,
      direct_approach_distance_, direct_approach_kp_);
  }

  // 生命周期：清理资源，直接委托给内部控制器
  void cleanup() override
  {
    inner_controller_->cleanup();
  }

  // 生命周期：激活控制器，直接委托给内部控制器
  void activate() override
  {
    inner_controller_->activate();
  }

  // 生命周期：停用控制器，直接委托给内部控制器
  void deactivate() override
  {
    inner_controller_->deactivate();
  }

  // 接收全局路径规划结果，缓存最后一个路径点作为目标位姿
  void setPlan(const nav_msgs::msg::Path & path) override
  {
    if (!path.poses.empty()) {
      goal_ = path.poses.back();
      if (goal_.header.frame_id.empty()) {
        goal_.header.frame_id = path.header.frame_id;
      }
    }
    inner_controller_->setPlan(path);
  }

  // 核心逻辑：计算速度指令，在接近目标时介入减速
  //
  // 三层策略（按优先级从高到低）：
  //   1. 直接驱动区 (dist < direct_approach_distance_):
  //      绕过内部控制器，用 P 控制直接朝目标点推进，角速度强制归零。
  //      目标速度 = min(approach_velocity_, dist * direct_approach_kp_)，
  //      距离越近速度越低，到达后速度归零。
  //
  //   2. 常规减速区 (dist < approach_distance_):
  //      保留内部控制器的方向，但按比例缩放线速度和角速度，
  //      使合速度不超过 approach_velocity_，避免高速冲过目标点。
  //
  //   3. 正常行驶区 (dist >= approach_distance_):
  //      完全透传内部控制器的输出，不做任何干预。
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    // 先获取内部控制器（如 MPPI）的原始速度指令
    auto cmd = inner_controller_->computeVelocityCommands(pose, velocity, goal_checker);

    // 计算当前位姿到目标点的欧氏距离
    auto goal = goal_;
    if (goal.header.frame_id.empty() || pose.header.frame_id.empty()) {
      throw nav2_core::ControllerTFError("Goal and robot pose require frame IDs");
    }
    if (goal.header.frame_id != pose.header.frame_id) {
      goal.header.stamp = pose.header.stamp;
      try {
        goal = tf_->transform(goal, pose.header.frame_id);
      } catch (const tf2::TransformException & ex) {
        throw nav2_core::ControllerTFError(ex.what());
      }
    }
    double dx = goal.pose.position.x - pose.pose.position.x;
    double dy = goal.pose.position.y - pose.pose.position.y;
    double dist = std::hypot(dx, dy);

    if (dist < direct_approach_distance_) {
      // 近距离直接驱动模式：绕过 MPPI 的弧线输出，直接朝目标点走
      double target_speed = std::min(approach_velocity_, dist * direct_approach_kp_);
      if (dist > 0.01) {
        // 将目标速度按单位方向向量分解到 x、y 轴
        const double yaw = tf2::getYaw(pose.pose.orientation);
        cmd.twist.linear.x = target_speed * (std::cos(yaw) * dx + std::sin(yaw) * dy) / dist;
        cmd.twist.linear.y = target_speed * (-std::sin(yaw) * dx + std::cos(yaw) * dy) / dist;
      } else {
        // 已到达目标点（0.01m 内），停止运动
        cmd.twist.linear.x = 0.0;
        cmd.twist.linear.y = 0.0;
      }
      // 强制角速度归零，确保机器人直线对准目标停止
      cmd.twist.angular.z = 0.0;
    } else if (dist < approach_distance_) {
      // 常规减速区：计算合速度并按比例缩放
      double speed = std::hypot(cmd.twist.linear.x, cmd.twist.linear.y);
      if (speed > approach_velocity_) {
        double scale = approach_velocity_ / speed;
        cmd.twist.linear.x *= scale;
        cmd.twist.linear.y *= scale;
        // 角速度也按相同比例降低，避免线速度降低后角速度相对过大导致原地打转
        cmd.twist.angular.z *= scale;
      }
    }
    // 否则 dist >= approach_distance_，正常行驶，不干预

    return cmd;
  }

  // 速度限制：直接委托给内部控制器处理
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    inner_controller_->setSpeedLimit(speed_limit, percentage);
  }

private:
  void setupDynamicParameters(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & name)
  {
    dyn_params_handler_ = node->add_on_set_parameters_callback(
      [this, name](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        double approach_distance = approach_distance_;
        double approach_velocity = approach_velocity_;
        double direct_approach_distance = direct_approach_distance_;
        double direct_approach_kp = direct_approach_kp_;

        for (const auto & parameter : parameters) {
          const auto & param_name = parameter.get_name();
          if (param_name == name + ".approach_distance") {
            approach_distance = parameter.as_double();
          } else if (param_name == name + ".approach_velocity") {
            approach_velocity = parameter.as_double();
          } else if (param_name == name + ".direct_approach_distance") {
            direct_approach_distance = parameter.as_double();
          } else if (param_name == name + ".direct_approach_kp") {
            direct_approach_kp = parameter.as_double();
          }
        }

        if (approach_distance <= 0.0 || approach_velocity < 0.0 ||
          direct_approach_distance < 0.0 || direct_approach_kp < 0.0)
        {
          result.successful = false;
          result.reason = "Goal approach parameters must be non-negative, and approach_distance must be positive";
          return result;
        }

        if (direct_approach_distance > approach_distance) {
          result.successful = false;
          result.reason = "direct_approach_distance must be <= approach_distance";
          return result;
        }

        approach_distance_ = approach_distance;
        approach_velocity_ = approach_velocity;
        direct_approach_distance_ = direct_approach_distance;
        direct_approach_kp_ = direct_approach_kp;
        return result;
      });
  }

  // 内部控制器实例（通过 pluginlib 动态加载，如 MPPI）
  pluginlib::UniquePtr<nav2_core::Controller> inner_controller_;
  // 插件类加载器（需保持生命周期与 inner_controller_ 一致）
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader_;
  // 动态参数回调句柄，保持生命周期以便运行时调参生效
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  // 日志器
  rclcpp::Logger logger_{rclcpp::get_logger("goal_approach_controller")};
  // 缓存的目标位姿，由 setPlan() 更新
  geometry_msgs::msg::PoseStamped goal_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  // 开始减速的距离阈值 (m)
  double approach_distance_{1.5};
  // 减速后的最大合速度 (m/s)
  double approach_velocity_{0.5};
  // 切换到直接驱动模式的距离阈值 (m)
  double direct_approach_distance_{0.5};
  // 直接驱动模式下的 P 控制增益
  double direct_approach_kp_{1.0};
};

}  // namespace goal_approach_controller

// 将 GoalApproachController 注册为 nav2_core::Controller 插件，
// 使 Nav2 控制器服务器可通过 pluginlib 动态加载本插件
PLUGINLIB_EXPORT_CLASS(
  goal_approach_controller::GoalApproachController,
  nav2_core::Controller)
