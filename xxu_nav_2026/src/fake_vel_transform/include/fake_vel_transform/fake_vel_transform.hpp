#ifndef FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_

#include <chrono>
#include <deque>
#include <mutex>

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace fake_vel_transform
{
class FakeVelTransform : public rclcpp::Node
{
public:
  explicit FakeVelTransform(const rclcpp::NodeOptions & options);

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  double yawAt(const rclcpp::Time & stamp) const;

  void publishCommand();

  bool odomIsFresh() const;

  struct OdomYawSample
  {
    rclcpp::Time stamp;
    double yaw;
  };

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_chassis_pub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr nav_odom_pub_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  geometry_msgs::msg::TwistStamped::SharedPtr last_command_;
  std::chrono::steady_clock::time_point last_command_receive_time_{};
  double command_timeout_{0.3};

  // Broadcast the stable velocity frame from the odometry child frame. The real
  // gimbal_link TF remains dynamic and is never replaced by this frame.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string robot_base_frame_;
  std::string fake_robot_base_frame_;
  std::string odom_topic_;
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  double spin_speed_;
  double gyro_linear_threshold_{0.01};
  double odom_history_duration_{2.0};
  double odom_timeout_{0.5};

  double current_robot_base_angle_{0.0};
  bool has_odom_{false};
  mutable std::mutex odom_mutex_;
  std::deque<OdomYawSample> odom_history_;
  std::chrono::steady_clock::time_point last_odom_receive_time_{};
};

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
