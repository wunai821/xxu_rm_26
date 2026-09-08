#include "xxu_gicp_localization/gicp_localization_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/registration/registration_result.hpp>
#include <tf2/exceptions.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace xxu_gicp_localization
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

bool finiteVector(const Eigen::Vector3d & vector)
{
  return vector.array().isFinite().all();
}

struct PcdField
{
  std::string name;
  int size{0};
  char type{'F'};
  int count{1};
  std::size_t offset{0};
};

bool parsePcdNumber(const std::string & token, double & value)
{
  try {
    std::size_t consumed = 0;
    value = std::stod(token, &consumed);
    return consumed == token.size() && std::isfinite(value);
  } catch (const std::exception &) {
    return false;
  }
}

bool readPcd(const std::string & path, std::vector<Eigen::Vector3d> & points, std::string & error)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    error = "cannot open file";
    return false;
  }

  std::vector<PcdField> fields;
  std::size_t points_count = 0;
  std::string data_encoding;
  std::streampos data_position = -1;
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream header(line);
    std::string key;
    header >> key;
    if (key == "FIELDS" || key == "FIELD") {
      fields.clear();
      std::string name;
      while (header >> name) {
        fields.push_back(PcdField{});
        fields.back().name = name;
      }
    } else if (key == "SIZE") {
      for (auto & field : fields) {
        header >> field.size;
      }
    } else if (key == "TYPE") {
      for (auto & field : fields) {
        header >> field.type;
      }
    } else if (key == "COUNT") {
      for (auto & field : fields) {
        header >> field.count;
      }
    } else if (key == "POINTS") {
      header >> points_count;
    } else if (key == "DATA") {
      header >> data_encoding;
      data_position = stream.tellg();
      break;
    }
  }

  if (data_position < 0 || fields.empty() || points_count == 0 ||
    (data_encoding != "ascii" && data_encoding != "binary"))
  {
    error = "requires FIELDS x y z, POINTS, and DATA ascii or DATA binary";
    return false;
  }

  std::size_t point_stride = 0;
  int x_index = -1;
  int y_index = -1;
  int z_index = -1;
  for (std::size_t index = 0; index < fields.size(); ++index) {
    auto & field = fields[index];
    if (field.size <= 0 || field.count <= 0) {
      error = "invalid field size/count";
      return false;
    }
    field.offset = point_stride;
    point_stride += static_cast<std::size_t>(field.size * field.count);
    if (field.name == "x") {
      x_index = static_cast<int>(index);
    } else if (field.name == "y") {
      y_index = static_cast<int>(index);
    } else if (field.name == "z") {
      z_index = static_cast<int>(index);
    }
  }
  if (x_index < 0 || y_index < 0 || z_index < 0) {
    error = "PCD does not contain x/y/z fields";
    return false;
  }

  points.clear();
  points.reserve(points_count);
  if (data_encoding == "ascii") {
    std::string data_line;
    while (points.size() < points_count && std::getline(stream, data_line)) {
      if (data_line.empty() || data_line[0] == '#') {
        continue;
      }
      std::istringstream values(data_line);
      std::vector<std::string> tokens;
      std::string token;
      while (values >> token) {
        tokens.push_back(token);
      }
      if (tokens.size() < fields.size()) {
        continue;
      }
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      if (!parsePcdNumber(tokens[static_cast<std::size_t>(x_index)], x) ||
        !parsePcdNumber(tokens[static_cast<std::size_t>(y_index)], y) ||
        !parsePcdNumber(tokens[static_cast<std::size_t>(z_index)], z))
      {
        continue;
      }
      points.emplace_back(x, y, z);
    }
    return !points.empty();
  }

  std::vector<std::uint8_t> record(point_stride);
  const auto read_value = [&record, &fields](int index, std::size_t point_offset, double & value) {
      const auto & field = fields[static_cast<std::size_t>(index)];
      const auto * bytes = record.data() + point_offset + field.offset;
      if (field.count != 1) {
        return false;
      }
      if (field.type == 'F' && field.size == 4) {
        float value_f = 0.0F;
        std::memcpy(&value_f, bytes, sizeof(value_f));
        value = value_f;
        return std::isfinite(value);
      }
      if (field.type == 'F' && field.size == 8) {
        std::memcpy(&value, bytes, sizeof(value));
        return std::isfinite(value);
      }
      if (field.type == 'I' && field.size == 4) {
        std::int32_t value_i = 0;
        std::memcpy(&value_i, bytes, sizeof(value_i));
        value = static_cast<double>(value_i);
        return true;
      }
      if (field.type == 'U' && field.size == 4) {
        std::uint32_t value_u = 0;
        std::memcpy(&value_u, bytes, sizeof(value_u));
        value = static_cast<double>(value_u);
        return true;
      }
      return false;
    };
  for (std::size_t index = 0; index < points_count; ++index) {
    if (!stream.read(reinterpret_cast<char *>(record.data()),
          static_cast<std::streamsize>(record.size())))
    {
      break;
    }
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (read_value(x_index, 0, x) && read_value(y_index, 0, y) && read_value(z_index, 0, z)) {
      points.emplace_back(x, y, z);
    }
  }
  return !points.empty();
}

}  // namespace

GicpLocalizationNode::GicpLocalizationNode(const rclcpp::NodeOptions & options)
: Node("gicp_localization", options)
{
  pcd_map_ = declare_parameter<std::string>("pcd_map", "");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  cloud_topic_ = declare_parameter<std::string>(
    "cloud_topic", "/mid360/livox_points_compensated");
  amcl_topic_ = declare_parameter<std::string>("amcl_topic", "/amcl_pose");
  pose_topic_ = declare_parameter<std::string>("pose_topic", "/gicp_pose");
  expected_cloud_frame_ = declare_parameter<std::string>("cloud_frame", "lio_base_sensor");
  downsampling_resolution_ = declare_parameter<double>("downsampling_resolution", 0.20);
  num_neighbors_ = declare_parameter<int>("num_neighbors", 20);
  num_threads_ = declare_parameter<int>("num_threads", 4);
  max_iterations_ = declare_parameter<int>("max_iterations", 32);
  max_correspondence_distance_ = declare_parameter<double>(
    "max_correspondence_distance", 1.0);
  tf_lookup_timeout_ = declare_parameter<double>("tf_lookup_timeout", 0.10);
  min_range_ = declare_parameter<double>("min_range", 0.5);
  max_range_ = declare_parameter<double>("max_range", 50.0);
  min_cloud_points_ = declare_parameter<int>("min_cloud_points", 80);
  min_inliers_ = declare_parameter<int>("min_inliers", 40);
  max_error_ = declare_parameter<double>("max_error", 1.0);
  max_pose_jump_ = declare_parameter<double>("max_pose_jump", 1.5);
  max_yaw_jump_ = declare_parameter<double>("max_yaw_jump", 1.2);
  max_relocalization_jump_ = declare_parameter<double>("max_relocalization_jump", 8.0);
  max_relocalization_yaw_ = declare_parameter<double>("max_relocalization_yaw", kPi);
  amcl_timeout_ = declare_parameter<double>("amcl_timeout", 2.0);
  cloud_timeout_ = declare_parameter<double>("cloud_timeout", 0.5);
  amcl_relocalization_distance_ = declare_parameter<double>(
    "amcl_relocalization_distance", 1.5);
  amcl_relocalization_yaw_ = declare_parameter<double>("amcl_relocalization_yaw", 1.0);
  output_covariance_xy_ = declare_parameter<double>("output_covariance_xy", 0.04);
  output_covariance_yaw_ = declare_parameter<double>("output_covariance_yaw", 0.04);

  if (downsampling_resolution_ <= 0.0 || num_neighbors_ < 3 || num_threads_ < 1 ||
    max_iterations_ < 1 || max_correspondence_distance_ <= 0.0 || min_range_ < 0.0 ||
    max_range_ <= min_range_ || min_cloud_points_ < 3 || min_inliers_ < 3 ||
    max_error_ <= 0.0 || max_pose_jump_ <= 0.0 || max_yaw_jump_ <= 0.0 ||
    amcl_timeout_ <= 0.0 || cloud_timeout_ <= 0.0)
  {
    throw std::invalid_argument("xxu_gicp_localization received an invalid parameter");
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  // lookupTransform uses a bounded timeout; tell tf2 that its listener has a
  // dedicated callback thread so the timeout can wait for incoming TF data.
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, true);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    pose_topic_, rclcpp::QoS(10).reliable());
  const auto amcl_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    amcl_topic_, amcl_qos,
    std::bind(&GicpLocalizationNode::amclCallback, this, std::placeholders::_1));
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&GicpLocalizationNode::cloudCallback, this, std::placeholders::_1));

  map_ready_ = loadMap();
  if (!map_ready_) {
    RCLCPP_ERROR(
      get_logger(),
      "No usable PCD map was loaded. GICP will stay inactive and will not publish map->odom.");
  }
  RCLCPP_INFO(
    get_logger(), "AMCL-seeded small_gicp localization started: cloud=%s map=%s",
      cloud_topic_.c_str(),
    pcd_map_.empty() ? "<unset>" : pcd_map_.c_str());
}

void GicpLocalizationNode::amclCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  const auto stamp = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
  const auto pose = planar(poseToEigen(msg->pose.pose));
  if (!finiteTransform(pose)) {
    RCLCPP_WARN(get_logger(), "Ignoring non-finite AMCL pose");
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  amcl_.map_base = pose;
  amcl_.stamp = stamp;
  amcl_.valid = true;
}

bool GicpLocalizationNode::loadMap()
{
  if (pcd_map_.empty()) {
    return false;
  }

  std::vector<Point> points;
  std::string pcd_error;
  if (!readPcd(pcd_map_, points, pcd_error)) {
    RCLCPP_ERROR(get_logger(), "Could not load PCD map %s: %s", pcd_map_.c_str(),
        pcd_error.c_str());
    return false;
  }
  points.erase(
    std::remove_if(points.begin(), points.end(), [](const Point & point) {
      return !finiteVector(point) || point.norm() > 10000.0;
    }), points.end());
  if (points.size() < static_cast<size_t>(min_inliers_)) {
    RCLCPP_ERROR(
      get_logger(), "PCD map contains only %zu usable points (need at least %d)",
      points.size(), min_inliers_);
    return false;
  }

  try {
    auto preprocessed = small_gicp::preprocess_points(
      points, downsampling_resolution_, num_neighbors_, num_threads_);
    target_points_ = std::move(preprocessed.first);
    target_tree_ = std::move(preprocessed.second);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "Could not preprocess PCD map: %s", exception.what());
    return false;
  }

  if (!target_points_ || !target_tree_ ||
    target_points_->size() < static_cast<size_t>(min_inliers_))
  {
    RCLCPP_ERROR(get_logger(), "PCD map is too sparse after downsampling");
    target_points_.reset();
    target_tree_.reset();
    return false;
  }
  RCLCPP_INFO(
    get_logger(), "Loaded PCD map: %zu points, %zu downsampled target points",
    points.size(), target_points_->size());
  return true;
}

bool GicpLocalizationNode::cloudToPoints(
  const sensor_msgs::msg::PointCloud2 & msg, std::vector<Point> & points) const
{
  points.clear();
  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const Point point(*iter_x, *iter_y, *iter_z);
      const double range = point.norm();
      if (finiteVector(point) && range >= min_range_ && range <= max_range_) {
        points.push_back(point);
      }
    }
  } catch (const std::exception & exception) {
    RCLCPP_WARN(get_logger(), "Ignoring PointCloud2 without x/y/z fields: %s", exception.what());
    return false;
  }
  return points.size() >= static_cast<size_t>(min_cloud_points_);
}

bool GicpLocalizationNode::lookupTransform(
  const std::string & target, const std::string & source, const rclcpp::Time & stamp,
  Transform & result) const
{
  const auto convert = [&result](const geometry_msgs::msg::TransformStamped & tf) {
      result = Transform::Identity();
      const auto & translation = tf.transform.translation;
      const auto & rotation = tf.transform.rotation;
      Eigen::Quaterniond quaternion(rotation.w, rotation.x, rotation.y, rotation.z);
      if (quaternion.norm() < std::numeric_limits<double>::epsilon()) {
        return false;
      }
      result.linear() = quaternion.normalized().toRotationMatrix();
      result.translation() = Eigen::Vector3d(translation.x, translation.y, translation.z);
      return finiteTransform(result);
    };
  try {
    const auto tf = tf_buffer_->lookupTransform(
      target, source, stamp, rclcpp::Duration::from_seconds(tf_lookup_timeout_));
    return convert(tf);
  } catch (const tf2::TransformException & exception) {
    try {
      const auto latest = tf_buffer_->lookupTransform(
        target, source, rclcpp::Time(0, 0, get_clock()->get_clock_type()),
        rclcpp::Duration::from_seconds(tf_lookup_timeout_));
      if (convert(latest)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Exact-time TF unavailable (%s <- %s); using latest transform: %s",
          target.c_str(), source.c_str(), exception.what());
        return true;
      }
    } catch (const tf2::TransformException &) {
      // Report the original timestamped lookup error below.
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "TF lookup failed (%s <- %s): %s",
      target.c_str(), source.c_str(), exception.what());
    return false;
  }
}

bool GicpLocalizationNode::amclIsFresh(const rclcpp::Time & now, const AmclState & amcl) const
{
  if (!amcl.valid) {
    return false;
  }
  if (amcl.stamp.nanoseconds() == 0) {
    return true;
  }
  const double age = (now - amcl.stamp).seconds();
  return age >= -0.5 && age <= amcl_timeout_;
}

bool GicpLocalizationNode::cloudIsFresh(const rclcpp::Time & now, const rclcpp::Time & stamp) const
{
  if (stamp.nanoseconds() == 0) {
    return true;
  }
  const double age = (now - stamp).seconds();
  return age >= -0.5 && age <= cloud_timeout_;
}

GicpLocalizationNode::Transform GicpLocalizationNode::poseToEigen(
  const geometry_msgs::msg::Pose & pose)
{
  Transform result = Transform::Identity();
  Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y,
    pose.orientation.z);
  if (quaternion.norm() > std::numeric_limits<double>::epsilon()) {
    result.linear() = quaternion.normalized().toRotationMatrix();
  }
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return result;
}

geometry_msgs::msg::Pose GicpLocalizationNode::eigenToPose(const Transform & transform)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Quaterniond quaternion(transform.rotation());
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

geometry_msgs::msg::Transform GicpLocalizationNode::eigenToTransform(const Transform & transform)
{
  geometry_msgs::msg::Transform message;
  const Eigen::Quaterniond quaternion(transform.rotation());
  message.translation.x = transform.translation().x();
  message.translation.y = transform.translation().y();
  message.translation.z = transform.translation().z();
  message.rotation.x = quaternion.x();
  message.rotation.y = quaternion.y();
  message.rotation.z = quaternion.z();
  message.rotation.w = quaternion.w();
  return message;
}

GicpLocalizationNode::Transform GicpLocalizationNode::planar(const Transform & transform)
{
  Transform result = Transform::Identity();
  const double yaw = std::atan2(transform.linear()(1, 0), transform.linear()(0, 0));
  result.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  result.translation() = Eigen::Vector3d(transform.translation().x(), transform.translation().y(),
      0.0);
  return result;
}

double GicpLocalizationNode::planarDistance(const Transform & lhs, const Transform & rhs)
{
  return (lhs.translation().head<2>() - rhs.translation().head<2>()).norm();
}

double GicpLocalizationNode::planarYawDistance(const Transform & lhs, const Transform & rhs)
{
  const double lhs_yaw = std::atan2(lhs.linear()(1, 0), lhs.linear()(0, 0));
  const double rhs_yaw = std::atan2(rhs.linear()(1, 0), rhs.linear()(0, 0));
          return std::abs(normalizeAngle(lhs_yaw - rhs_yaw));
}

bool GicpLocalizationNode::finiteTransform(const Transform & transform)
{
  return transform.matrix().array().isFinite().all();
}

bool GicpLocalizationNode::acceptResult(
  const Transform & map_base, const Transform & reference_base,
  const small_gicp::RegistrationResult & result, bool reinitialized) const
{
  const double mean_error = result.num_inliers == 0 ?
    std::numeric_limits<double>::infinity() : result.error / result.num_inliers;
  if (!result.converged || result.num_inliers < static_cast<size_t>(min_inliers_) ||
    !std::isfinite(mean_error) || mean_error > max_error_ || !finiteTransform(map_base))
  {
    return false;
  }
  const double jump = planarDistance(map_base, reference_base);
  const double yaw_jump = planarYawDistance(map_base, reference_base);
  if (reinitialized) {
    return jump <= max_relocalization_jump_ && yaw_jump <= max_relocalization_yaw_;
  }
  return jump <= max_pose_jump_ && yaw_jump <= max_yaw_jump_;
}

void GicpLocalizationNode::publishResult(
  const Transform & map_base, const Transform & odom_base, const rclcpp::Time & stamp,
  const small_gicp::RegistrationResult & result)
{
  const Transform map_odom = planar(map_base * planar(odom_base).inverse());

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = map_frame_;
  pose.pose.pose = eigenToPose(map_base);
  pose.pose.covariance[0] = output_covariance_xy_;
  pose.pose.covariance[7] = output_covariance_xy_;
  pose.pose.covariance[35] = output_covariance_yaw_;
  pose_pub_->publish(pose);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = map_frame_;
  transform.child_frame_id = odom_frame_;
  transform.transform = eigenToTransform(map_odom);
  tf_broadcaster_->sendTransform(transform);

  RCLCPP_DEBUG(
    get_logger(), "Accepted GICP result: inliers=%zu mean_error=%.4f iterations=%zu",
    result.num_inliers, result.error / std::max<size_t>(result.num_inliers, 1), result.iterations);
}

void GicpLocalizationNode::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!msg || !map_ready_) {
    return;
  }
  const std::string source_frame = msg->header.frame_id.empty() ? expected_cloud_frame_ :
    msg->header.frame_id;
  if (source_frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Dropping cloud with no frame_id");
    return;
  }
  if (!expected_cloud_frame_.empty() && source_frame != expected_cloud_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Dropping cloud from unexpected frame %s (expected %s)",
      source_frame.c_str(), expected_cloud_frame_.c_str());
    return;
  }

  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const rclcpp::Time now = get_clock()->now();
  if (!cloudIsFresh(now, stamp)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Dropping stale cloud (age %.3f s)",
      (now - stamp).seconds());
    return;
  }

  std::vector<Point> source_points;
  if (!cloudToPoints(*msg, source_points)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Dropping cloud with fewer than %d usable points",
      min_cloud_points_);
    return;
  }

  AmclState amcl;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    amcl = amcl_;
  }
  const bool fresh_amcl = amclIsFresh(now, amcl);
  if (!have_last_pose_ && !fresh_amcl) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for a fresh AMCL pose before GICP");
    return;
  }

  Transform odom_base;
  Transform base_sensor;
  if (!lookupTransform(odom_frame_, base_frame_, stamp, odom_base) ||
    !lookupTransform(base_frame_, source_frame, stamp, base_sensor))
  {
    return;
  }
  odom_base = planar(odom_base);

  Transform initial_map_base = amcl.map_base;
  bool reinitialized = !have_last_pose_;
  if (have_last_pose_) {
    const Transform predicted_map_base = planar(last_map_base_ * last_odom_base_.inverse() *
        odom_base);
    const bool amcl_jump = fresh_amcl &&
      (planarDistance(amcl.map_base, predicted_map_base) > amcl_relocalization_distance_ ||
      planarYawDistance(amcl.map_base, predicted_map_base) > amcl_relocalization_yaw_);
    initial_map_base = amcl_jump ? amcl.map_base : predicted_map_base;
    reinitialized = amcl_jump;
  }
  const Transform initial_map_sensor = planar(initial_map_base) * base_sensor;

  small_gicp::RegistrationResult result;
  try {
    auto source_preprocessed = small_gicp::preprocess_points(
      source_points, downsampling_resolution_, num_neighbors_, num_threads_);
    if (!source_preprocessed.first || source_preprocessed.first->size() <
      static_cast<size_t>(min_inliers_))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cloud became too sparse after downsampling");
      return;
    }
    small_gicp::RegistrationSetting setting;
    setting.type = small_gicp::RegistrationSetting::GICP;
    setting.downsampling_resolution = downsampling_resolution_;
    setting.max_correspondence_distance = max_correspondence_distance_;
    setting.max_iterations = max_iterations_;
    setting.num_threads = num_threads_;
    setting.rotation_eps = 0.001;
    setting.translation_eps = 0.001;
    result = small_gicp::align(
      *target_points_, *source_preprocessed.first, *target_tree_, initial_map_sensor, setting);
  } catch (const std::exception & exception) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "small_gicp failed: %s", exception.what());
    return;
  }

  const Transform map_base = planar(result.T_target_source * base_sensor.inverse());
  const Transform reference = have_last_pose_ && !reinitialized ?
    planar(last_map_base_ * last_odom_base_.inverse() * odom_base) : amcl.map_base;
  if (!acceptResult(map_base, reference, result, reinitialized)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Rejected GICP result: converged=%s inliers=%zu mean_error=%.4f jump=%.3f "
      "yaw_jump=%.3f",
      result.converged ? "true" : "false", result.num_inliers,
      result.error / std::max<size_t>(result.num_inliers, 1),
      planarDistance(map_base, reference), planarYawDistance(map_base, reference));
    return;
  }

  publishResult(map_base, odom_base, stamp, result);
  last_map_base_ = map_base;
  last_odom_base_ = odom_base;
  have_last_pose_ = true;
}

}  // namespace xxu_gicp_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xxu_gicp_localization::GicpLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
