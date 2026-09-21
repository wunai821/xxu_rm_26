#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration_result.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>
#include <cstddef>
#include <vector>

namespace xxu_gicp_localization
{

class GicpLocalizationNode final : public rclcpp::Node {
public:
  explicit GicpLocalizationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GicpLocalizationNode() override;

private:
  using Point = Eigen::Vector3d;
  using Transform = Eigen::Isometry3d;

  struct AmclState
  {
    Transform map_base{Transform::Identity()};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  bool loadMap();
  bool cloudToPoints(
    const sensor_msgs::msg::PointCloud2 & msg, std::vector<Point> & points) const;
  bool lookupTransform(
    const std::string & target, const std::string & source, const rclcpp::Time & stamp,
    Transform & result) const;
  bool amclIsFresh(const rclcpp::Time & now, const AmclState & amcl) const;
  bool cloudIsFresh(const rclcpp::Time & now, const rclcpp::Time & stamp) const;

  static Transform poseToEigen(const geometry_msgs::msg::Pose & pose);
  static geometry_msgs::msg::Pose eigenToPose(const Transform & transform);
  static geometry_msgs::msg::Transform eigenToTransform(const Transform & transform);
  static Transform planar(const Transform & transform);
  static double planarDistance(const Transform & lhs, const Transform & rhs);
  static double planarYawDistance(const Transform & lhs, const Transform & rhs);
  static bool finiteTransform(const Transform & transform);

  bool acceptResult(
    const Transform & map_base, const Transform & reference_base,
    const struct small_gicp::RegistrationResult & result, bool reinitialized) const;
  void publishResult(
    const Transform & map_base, const Transform & odom_base,
    const rclcpp::Time & stamp, const struct small_gicp::RegistrationResult & result);

  // ROS interfaces.
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // Preprocessed map used as the fixed GICP target.
  small_gicp::PointCloud::Ptr target_points_;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> target_tree_;

  // Parameters.
  std::string pcd_map_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string cloud_topic_;
  std::string amcl_topic_;
  std::string pose_topic_;
  std::string expected_cloud_frame_;
  double downsampling_resolution_{0.2};
  int num_neighbors_{20};
  int num_threads_{4};
  int max_iterations_{32};
  double max_correspondence_distance_{1.0};
  double tf_lookup_timeout_{0.25};
  double min_range_{0.5};
  double max_range_{50.0};
  int min_cloud_points_{80};
  int min_inliers_{40};
  double max_error_{1.0};
  double max_pose_jump_{1.5};
  double max_yaw_jump_{1.2};
  double max_relocalization_jump_{8.0};
  double max_relocalization_yaw_{M_PI};
  double amcl_timeout_{2.0};
  double cloud_timeout_{0.5};
  double amcl_relocalization_distance_{1.5};
  double amcl_relocalization_yaw_{1.0};
  double output_covariance_xy_{0.04};
  double output_covariance_yaw_{0.04};

  mutable std::mutex state_mutex_;
  AmclState amcl_;
  Transform last_map_base_{Transform::Identity()};
  Transform last_odom_base_{Transform::Identity()};
  bool have_last_pose_{false};
  bool map_ready_{false};

  // Registration counters are emitted once at shutdown so throttled warning
  // logs cannot hide how many clouds were actually attempted or rejected.
  std::size_t clouds_received_{0};
  std::size_t dropped_map_not_ready_{0};
  std::size_t dropped_no_frame_{0};
  std::size_t dropped_unexpected_frame_{0};
  std::size_t dropped_stale_{0};
  std::size_t dropped_cloud_points_{0};
  std::size_t dropped_no_amcl_{0};
  std::size_t dropped_tf_{0};
  std::size_t dropped_preprocess_sparse_{0};
  std::size_t registration_attempts_{0};
  std::size_t converged_results_{0};
  std::size_t accepted_results_{0};
  std::size_t rejected_not_converged_{0};
  std::size_t rejected_inliers_{0};
  std::size_t rejected_error_{0};
  std::size_t rejected_jump_{0};
  std::size_t rejected_yaw_jump_{0};
  std::size_t registration_exceptions_{0};
};

}  // namespace xxu_gicp_localization
