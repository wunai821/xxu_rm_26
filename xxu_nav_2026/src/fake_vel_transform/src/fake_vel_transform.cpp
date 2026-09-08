#include "fake_vel_transform/fake_vel_transform.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/utils.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fake_vel_transform
{
FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "base_link");
  // The fake frame is intentionally a chassis child rather than a child of
  // gimbal_link.  The latter must remain the real, rotating sensor TF.
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_yaw_fake");
  this->declare_parameter<std::string>("odom_topic", "Odometry");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "cmd_vel");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "aft_cmd_vel");
  this->declare_parameter<float>("spin_speed", 0.0);
  this->declare_parameter<double>("gyro_linear_threshold", 0.01);
  this->declare_parameter<double>("odom_history_duration", 2.0);
  this->declare_parameter<double>("odom_timeout", 0.5);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("spin_speed", spin_speed_);
  this->get_parameter("gyro_linear_threshold", gyro_linear_threshold_);
  if (gyro_linear_threshold_ < 0.0) {
    gyro_linear_threshold_ = 0.0;
  }
  this->get_parameter("odom_history_duration", odom_history_duration_);
  if (odom_history_duration_ <= 0.0) {
    odom_history_duration_ = 2.0;
  }
  this->get_parameter("odom_timeout", odom_timeout_);
  if (odom_timeout_ <= 0.0) {
    odom_timeout_ = 0.5;
  }

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::TwistStamped>(output_cmd_vel_topic_, 1);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    input_cmd_vel_topic_, 1,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&FakeVelTransform::odomCallback, this, std::placeholders::_1));
}

void FakeVelTransform::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // /odom is published for base_footprint by Small Point-LIO after applying
  // the time-matched base<->LiDAR TF.  Its yaw is therefore the chassis yaw,
  // not the instantaneous gimbal joint angle.
  const rclcpp::Time stamp(msg->header.stamp);
  const double yaw = tf2::getYaw(msg->pose.pose.orientation);

  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    last_odom_receive_time_ = std::chrono::steady_clock::now();
    current_robot_base_angle_ = yaw;
    has_odom_ = true;

    if (odom_history_.empty() || stamp > odom_history_.back().stamp) {
      odom_history_.push_back({stamp, yaw});
    } else if (stamp == odom_history_.back().stamp) {
      odom_history_.back().yaw = yaw;
    }

    while (odom_history_.size() > 2 &&
           (stamp - odom_history_.front().stamp).seconds() > odom_history_duration_) {
      odom_history_.pop_front();
    }
  }

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = msg->header.stamp;
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  q.setRPY(0, 0, -current_robot_base_angle_);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

double FakeVelTransform::yawAt(const rclcpp::Time & stamp) const
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (!has_odom_ || odom_history_.empty() || stamp.nanoseconds() == 0) {
    return current_robot_base_angle_;
  }

  if (stamp <= odom_history_.front().stamp) {
    return odom_history_.front().yaw;
  }
  if (stamp >= odom_history_.back().stamp) {
    return odom_history_.back().yaw;
  }

  for (std::size_t i = 1; i < odom_history_.size(); ++i) {
    const auto & before = odom_history_[i - 1];
    const auto & after = odom_history_[i];
    if (stamp <= after.stamp) {
      const double span = (after.stamp - before.stamp).seconds();
      if (span <= 0.0) {
        return after.yaw;
      }
      const double ratio = (stamp - before.stamp).seconds() / span;
      const double delta = std::atan2(
        std::sin(after.yaw - before.yaw), std::cos(after.yaw - before.yaw));
      return before.yaw + ratio * delta;
    }
  }

  return odom_history_.back().yaw;
}

bool FakeVelTransform::odomIsFresh() const
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (!has_odom_) {
    return false;
  }
  const auto age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_odom_receive_time_).count();
  return age >= 0.0 && age <= odom_timeout_;
}

// Transform linear velocity from the fake base frame to the robot base frame.
// Keep angular.z from the incoming command so teleop can toggle gyro mode.
void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  // A stale odometry stream means the frame used to rotate the command is no
  // longer trustworthy.  Fail closed by withholding the transformed command;
  // the downstream watchdog will publish an explicit zero command.
  if (!odomIsFresh()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Withholding velocity command because odometry is missing or stale");
    return;
  }

  // Use the command's generation time rather than callback time.  This keeps
  // the velocity rotation synchronized with the odom/TF sample that Nav2 used.
  const double angle_diff = yawAt(rclcpp::Time(msg->header.stamp));

  geometry_msgs::msg::TwistStamped aft_tf_vel = *msg;
  aft_tf_vel.header.frame_id = robot_base_frame_;
  const double linear_speed = std::hypot(
    msg->twist.linear.x, msg->twist.linear.y);
  const bool is_moving = linear_speed > gyro_linear_threshold_;
  aft_tf_vel.twist.angular.z = msg->twist.angular.z +
    (is_moving ? static_cast<double>(spin_speed_) : 0.0);
  aft_tf_vel.twist.linear.x =
    msg->twist.linear.x * std::cos(angle_diff) + msg->twist.linear.y * std::sin(angle_diff);
  aft_tf_vel.twist.linear.y =
    -msg->twist.linear.x * std::sin(angle_diff) + msg->twist.linear.y * std::cos(angle_diff);

  cmd_vel_chassis_pub_->publish(aft_tf_vel);
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
