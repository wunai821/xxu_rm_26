#include "fake_vel_transform/fake_vel_transform.hpp"

#include <cmath>
#include <stdexcept>

#include <tf2/utils.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fake_vel_transform
{
FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  robot_base_frame_ = this->declare_parameter<std::string>("robot_base_frame", "base_link");
  // The fake frame is intentionally a chassis child rather than a child of
  // gimbal_link.  The latter must remain the real, rotating sensor TF.
  fake_robot_base_frame_ = this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_yaw_fake");
  const auto odom_topic = this->declare_parameter<std::string>("odom_topic", "Odometry");
  const auto input_cmd_vel_topic = this->declare_parameter<std::string>("input_cmd_vel_topic", "cmd_vel");
  const auto output_cmd_vel_topic = this->declare_parameter<std::string>("output_cmd_vel_topic", "aft_cmd_vel");
  spin_speed_ = this->declare_parameter<double>("spin_speed", 0.0);
  gyro_linear_threshold_ = this->declare_parameter<double>("gyro_linear_threshold", 0.01);
  odom_timeout_ = this->declare_parameter<double>("odom_timeout", 0.5);
  const auto velocity_window = this->declare_parameter<double>("nav_velocity_smoothing_window", 0.0);
  nav_velocity_window_ = std::make_unique<VelocityWindow>(velocity_window);
  const auto nav_odom_topic = this->declare_parameter<std::string>("nav_odom_topic", "/odom_nav");
  command_timeout_ = this->declare_parameter<double>("command_timeout", 0.3);
  const auto publish_rate = this->declare_parameter<double>("publish_rate", 100.0);
  if (command_timeout_ <= 0.0 || !std::isfinite(command_timeout_) ||
    publish_rate <= 0.0 || !std::isfinite(publish_rate))
  {
    throw std::invalid_argument("command_timeout and publish_rate must be finite and positive");
  }

  if (gyro_linear_threshold_ < 0.0) {
    gyro_linear_threshold_ = 0.0;
  }
  if (!std::isfinite(odom_timeout_) || odom_timeout_ <= 0.0) {
    odom_timeout_ = 0.5;
  }
  if (!std::isfinite(spin_speed_)) {
    throw std::invalid_argument("spin_speed must be finite");
  }
  nav_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(nav_odom_topic, 10);
  command_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate),
    std::bind(&FakeVelTransform::publishCommand, this));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::TwistStamped>(output_cmd_vel_topic, 1);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    input_cmd_vel_topic, 1,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10, std::bind(&FakeVelTransform::odomCallback, this, std::placeholders::_1));

  parameter_callback_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        if (name != "spin_speed" ||
          parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
          !std::isfinite(parameter.as_double()))
        {
          result.successful = false;
          result.reason = "Only finite spin_speed is dynamic";
          return result;
        }
      }
      for (const auto & parameter : parameters) {
        spin_speed_ = parameter.as_double();
      }
      return result;
    });
}

void FakeVelTransform::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // /odom is published for base_footprint by Small Point-LIO after applying
  // the time-matched base<->LiDAR TF.  Its yaw is therefore the chassis yaw,
  // not the instantaneous gimbal joint angle.
  const rclcpp::Time stamp(msg->header.stamp, this->get_clock()->get_clock_type());
  const double stamp_age = (this->now() - stamp).seconds();
  const double yaw = tf2::getYaw(msg->pose.pose.orientation);
  if (stamp_age < -0.1 || stamp_age > odom_timeout_ || !std::isfinite(yaw) ||
    !std::isfinite(msg->twist.twist.linear.x) || !std::isfinite(msg->twist.twist.linear.y))
  {
    return;
  }
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  VelocityWindow::Velocity nav_velocity;

  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    // Replayed/out-of-order samples must not refresh the watchdog or heading.
    if (has_odom_ && stamp <= latest_odom_.stamp) {
      return;
    }
    if (velocity_parent_frame_ != msg->header.frame_id ||
      velocity_child_frame_ != msg->child_frame_id)
    {
      nav_velocity_window_->reset();
      has_odom_ = false;
      velocity_parent_frame_ = msg->header.frame_id;
      velocity_child_frame_ = msg->child_frame_id;
    }
    // Average only after rotating into the yaw-stable odom/navigation frame.
    // Averaging rotating chassis components would mix different directions.
    if (!nav_velocity_window_->update(
        stamp.seconds(), c * msg->twist.twist.linear.x - s * msg->twist.twist.linear.y,
        s * msg->twist.twist.linear.x + c * msg->twist.twist.linear.y, nav_velocity))
    {
      return;
    }
    last_odom_receive_time_ = std::chrono::steady_clock::now();

    latest_odom_ = {stamp, yaw};
    has_odom_ = true;
  }

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = msg->header.stamp;
  t.header.frame_id = msg->child_frame_id;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  q = q.inverse();
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);

  // Planar navigation frame: same XY origin, yaw fixed in odom. The raw
  // odometry remains untouched for localization and real chassis feedback.
  auto nav_odom = *msg;
  nav_odom.child_frame_id = fake_robot_base_frame_;
  nav_odom.pose.pose.orientation = geometry_msgs::msg::Quaternion();
  nav_odom.pose.pose.orientation.w = 1.0;
  nav_odom.twist.twist.linear.x = nav_velocity.x;
  nav_odom.twist.twist.linear.y = nav_velocity.y;
  nav_odom.twist.twist.linear.z = 0.0;
  nav_odom.twist.twist.angular = geometry_msgs::msg::Vector3();
  // Project covariance onto the planar virtual frame. Its orientation is a
  // definition, not an estimate of the physical chassis orientation.
  nav_odom.pose.covariance.fill(0.0);
  nav_odom.twist.covariance.fill(0.0);
  const double rotation[2][2] = {{c, -s}, {s, c}};
  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      nav_odom.pose.covariance[i * 6 + j] = msg->pose.covariance[i * 6 + j];
      for (size_t k = 0; k < 2; ++k) {
        for (size_t l = 0; l < 2; ++l) {
          nav_odom.twist.covariance[i * 6 + j] +=
            rotation[i][k] * msg->twist.covariance[k * 6 + l] * rotation[j][l];
        }
      }
    }
  }
  nav_odom_pub_->publish(nav_odom);
}

bool FakeVelTransform::currentYaw(double & yaw) const
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (!has_odom_) {
    return false;
  }
  const auto age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_odom_receive_time_).count();
  const double stamp_age = (this->now() - latest_odom_.stamp).seconds();
  yaw = latest_odom_.yaw;
  return age >= 0.0 && age <= odom_timeout_ &&
         stamp_age >= -0.1 && stamp_age <= odom_timeout_;
}

// Navigation commands describe translation in the fixed odom-aligned frame.
// Nav2 is configured with zero yaw commands; explicit maintenance commands
// may still request physical rotation.
void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (msg->header.frame_id != fake_robot_base_frame_ ||
    !std::isfinite(msg->twist.linear.x) || !std::isfinite(msg->twist.linear.y) ||
    !std::isfinite(msg->twist.angular.z))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Rejecting navigation command with invalid frame or velocity");
    return;
  }
  last_command_ = msg;
  last_command_receive_time_ = std::chrono::steady_clock::now();
  publishCommand();
}

void FakeVelTransform::publishCommand()
{
  if (!last_command_) {
    return;
  }
  geometry_msgs::msg::TwistStamped output;
  output.header.stamp = this->now();
  output.header.frame_id = robot_base_frame_;
  const double receive_age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_command_receive_time_).count();
  const double stamp_age = (this->now() - rclcpp::Time(last_command_->header.stamp)).seconds();
  double yaw = 0.0;
  if (!currentYaw(yaw) || receive_age > command_timeout_ ||
    stamp_age < 0.0 || stamp_age > command_timeout_)
  {
    cmd_vel_chassis_pub_->publish(output);
    return;
  }
  const auto & input = last_command_->twist;
  // A collision/zero command stops all motion, including chassis spin.
  if (input.linear.x == 0.0 && input.linear.y == 0.0 && input.angular.z == 0.0) {
    cmd_vel_chassis_pub_->publish(output);
    return;
  }
  output.twist.linear.x = input.linear.x * std::cos(yaw) + input.linear.y * std::sin(yaw);
  output.twist.linear.y = -input.linear.x * std::sin(yaw) + input.linear.y * std::cos(yaw);
  // Zero / collision-stop commands always stop spin as well. A mission node
  // can change spin_speed through the ROS parameter service without a human.
  output.twist.angular.z = input.angular.z;
  if (std::hypot(input.linear.x, input.linear.y) > gyro_linear_threshold_) {
    output.twist.angular.z += spin_speed_;
  }
  cmd_vel_chassis_pub_->publish(output);

}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
