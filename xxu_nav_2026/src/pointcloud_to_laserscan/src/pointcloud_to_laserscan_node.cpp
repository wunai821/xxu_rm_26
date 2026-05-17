/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include "pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace pointcloud_to_laserscan
{
namespace
{
struct PointFields
{
  uint32_t x_offset{0};
  uint32_t y_offset{0};
  uint32_t z_offset{0};
};

bool findXYZFields(const sensor_msgs::msg::PointCloud2 & msg, PointFields & fields)
{
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;

  for (const auto & field : msg.fields) {
    const bool valid_float =
      field.datatype == sensor_msgs::msg::PointField::FLOAT32 && field.count == 1;

    if (field.name == "x" && valid_float) {
      fields.x_offset = field.offset;
      has_x = true;
    } else if (field.name == "y" && valid_float) {
      fields.y_offset = field.offset;
      has_y = true;
    } else if (field.name == "z" && valid_float) {
      fields.z_offset = field.offset;
      has_z = true;
    }
  }

  return has_x && has_y && has_z;
}

bool isHostBigendian()
{
  const uint16_t value = 0x0102;
  return *reinterpret_cast<const uint8_t *>(&value) == 0x01;
}

bool readFloat32(
  const std::vector<uint8_t> & data,
  const size_t offset,
  const bool is_bigendian,
  float & value)
{
  if (offset + sizeof(float) > data.size()) {
    return false;
  }

  uint8_t bytes[sizeof(float)];
  std::memcpy(bytes, data.data() + offset, sizeof(float));
  if (isHostBigendian() != is_bigendian) {
    std::reverse(bytes, bytes + sizeof(float));
  }
  std::memcpy(&value, bytes, sizeof(float));
  return true;
}

bool writeFloat32(
  std::vector<uint8_t> & data,
  const size_t offset,
  const bool is_bigendian,
  const float value)
{
  if (offset + sizeof(float) > data.size()) {
    return false;
  }

  uint8_t bytes[sizeof(float)];
  std::memcpy(bytes, &value, sizeof(float));
  if (isHostBigendian() != is_bigendian) {
    std::reverse(bytes, bytes + sizeof(float));
  }
  std::memcpy(data.data() + offset, bytes, sizeof(float));
  return true;
}
}  // namespace

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_laserscan", options)
{
  target_frame_ = this->declare_parameter("target_frame", "");
  tolerance_ = this->declare_parameter("transform_tolerance", 0.01);
  // TODO(hidmic): adjust default input queue size based on actual concurrency levels
  // achievable by the associated executor
  input_queue_size_ = this->declare_parameter(
    "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter("use_inf", true);
  publish_processed_cloud_ = this->declare_parameter("publish_processed_cloud", false);
  processed_cloud_frame_ = this->declare_parameter("processed_cloud_frame", target_frame_);
  processed_cloud_project_to_2d_ =
    this->declare_parameter("processed_cloud_project_to_2d", true);
  processed_cloud_z_value_ = this->declare_parameter("processed_cloud_z_value", 0.0);
  crop_min_x_ = this->declare_parameter("crop_min_x", -std::numeric_limits<double>::max());
  crop_max_x_ = this->declare_parameter("crop_max_x", std::numeric_limits<double>::max());
  crop_min_y_ = this->declare_parameter("crop_min_y", -std::numeric_limits<double>::max());
  crop_max_y_ = this->declare_parameter("crop_max_y", std::numeric_limits<double>::max());
  const auto scan_qos_reliability =
    this->declare_parameter<std::string>("scan_qos_reliability", "reliable");

  auto scan_qos = rclcpp::QoS(10);
  if (scan_qos_reliability == "best_effort") {
    scan_qos.best_effort();
  } else {
    scan_qos.reliable();
  }

  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", scan_qos);
  RCLCPP_INFO(
    this->get_logger(), "Publishing LaserScan with %s QoS",
    scan_qos_reliability == "best_effort" ? "best_effort" : "reliable");

  if (publish_processed_cloud_) {
    processed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "processed_cloud", rclcpp::QoS(10).reliable());
    RCLCPP_INFO(
      this->get_logger(),
      "Publishing processed cloud frame=%s, crop_x=[%.3f, %.3f], crop_y=[%.3f, %.3f], "
      "height=[%.3f, %.3f], range=[%.3f, %.3f], project_to_2d=%s",
      processed_cloud_frame_.c_str(), crop_min_x_, crop_max_x_, crop_min_y_, crop_max_y_,
      min_height_, max_height_, range_min_, range_max_,
      processed_cloud_project_to_2d_ ? "true" : "false");
  }

  using std::placeholders::_1;
  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty()) {
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    message_filter_ = std::make_unique<MessageFilter>(
      sub_, *tf2_, target_frame_, input_queue_size_,
      this->get_node_logging_interface(),
      this->get_node_clock_interface());
    message_filter_->registerCallback(
      std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  } else {  // otherwise setup direct subscription
    sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  }

  subscription_listener_thread_ = std::thread(
    std::bind(&PointCloudToLaserScanNode::subscriptionListenerThreadLoop, this));
}

PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
{
  alive_.store(false);
  subscription_listener_thread_.join();
}

void PointCloudToLaserScanNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();

  const std::chrono::milliseconds timeout(100);
  while (rclcpp::ok(context) && alive_.load()) {
    int subscription_count = pub_->get_subscription_count() +
      pub_->get_intra_process_subscription_count();
    if (processed_cloud_pub_) {
      subscription_count += processed_cloud_pub_->get_subscription_count() +
        processed_cloud_pub_->get_intra_process_subscription_count();
    }
    if (subscription_count > 0) {
      if (!sub_.getSubscriber()) {
        RCLCPP_INFO(
          this->get_logger(),
          "Got a subscriber to laserscan, starting pointcloud subscriber");
        rclcpp::SensorDataQoS qos;
        qos.keep_last(input_queue_size_);
        sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
      }
    } else if (sub_.getSubscriber()) {
      RCLCPP_INFO(
        this->get_logger(),
        "No subscribers to laserscan, shutting down pointcloud subscriber");
      sub_.unsubscribe();
    }
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }
  sub_.unsubscribe();
}

void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  // build laserscan output
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;
  if (!target_frame_.empty()) {
    scan_msg->header.frame_id = target_frame_;
  }

  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  // Transform cloud if necessary
  if (scan_msg->header.frame_id != cloud_msg->header.frame_id) {
    try {
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_));
      cloud_msg = cloud;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
      return;
    }
  }

  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
    iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for nan in point(%f, %f, %f)\n",
        *iter_x, *iter_y, *iter_z);
      continue;
    }

    if (*iter_z > max_height_ || *iter_z < min_height_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for height %f not in range (%f, %f)\n",
        *iter_z, min_height_, max_height_);
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
        range, range_min_, *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
        range, range_max_, *iter_x, *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for angle %f not in range (%f, %f)\n",
        angle, scan_msg->angle_min, scan_msg->angle_max);
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
    if (range < scan_msg->ranges[index]) {
      scan_msg->ranges[index] = range;
    }
  }

  this->publishProcessedCloud(cloud_msg);

  pub_->publish(std::move(scan_msg));
}

void PointCloudToLaserScanNode::publishProcessedCloud(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  if (!publish_processed_cloud_ || !processed_cloud_pub_) {
    return;
  }

  auto processed_input = cloud_msg;
  if (!processed_cloud_frame_.empty() &&
    processed_cloud_frame_ != cloud_msg->header.frame_id)
  {
    try {
      auto transformed_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2_->transform(
        *cloud_msg, *transformed_cloud, processed_cloud_frame_,
        tf2::durationFromSec(tolerance_));
      processed_input = transformed_cloud;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Processed cloud transform failure: %s", ex.what());
      return;
    }
  }

  PointFields fields{};
  if (!findXYZFields(*processed_input, fields)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "PointCloud2 x/y/z fields must exist and be FLOAT32");
    return;
  }

  sensor_msgs::msg::PointCloud2 output;
  output.header = processed_input->header;
  if (!processed_cloud_frame_.empty()) {
    output.header.frame_id = processed_cloud_frame_;
  }
  output.height = 1;
  output.fields = processed_input->fields;
  output.is_bigendian = processed_input->is_bigendian;
  output.point_step = processed_input->point_step;
  output.is_dense = false;
  output.data.reserve(processed_input->data.size());

  for (uint32_t row = 0; row < processed_input->height; ++row) {
    const size_t row_start = static_cast<size_t>(row) * processed_input->row_step;
    for (uint32_t column = 0; column < processed_input->width; ++column) {
      const size_t point_start =
        row_start + static_cast<size_t>(column) * processed_input->point_step;
      if (point_start + processed_input->point_step > processed_input->data.size()) {
        continue;
      }

      float x = 0.0F;
      float y = 0.0F;
      float z = 0.0F;
      if (!readFloat32(
          processed_input->data, point_start + fields.x_offset,
          processed_input->is_bigendian, x) ||
        !readFloat32(
          processed_input->data, point_start + fields.y_offset,
          processed_input->is_bigendian, y) ||
        !readFloat32(
          processed_input->data, point_start + fields.z_offset,
          processed_input->is_bigendian, z))
      {
        continue;
      }

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      if (z < min_height_ || z > max_height_) {
        continue;
      }
      if (x < crop_min_x_ || x > crop_max_x_ || y < crop_min_y_ || y > crop_max_y_) {
        continue;
      }

      const double range = std::hypot(x, y);
      if (range < range_min_ || range > range_max_) {
        continue;
      }

      const size_t output_start = output.data.size();
      output.data.insert(
        output.data.end(),
        processed_input->data.begin() +
        static_cast<std::vector<uint8_t>::difference_type>(point_start),
        processed_input->data.begin() +
        static_cast<std::vector<uint8_t>::difference_type>(
          point_start + processed_input->point_step));

      writeFloat32(output.data, output_start + fields.x_offset, output.is_bigendian, x);
      writeFloat32(output.data, output_start + fields.y_offset, output.is_bigendian, y);
      writeFloat32(
        output.data, output_start + fields.z_offset, output.is_bigendian,
        processed_cloud_project_to_2d_ ? static_cast<float>(processed_cloud_z_value_) : z);
    }
  }

  output.width = static_cast<uint32_t>(output.data.size() / output.point_step);
  output.row_step = static_cast<uint32_t>(output.data.size());
  processed_cloud_pub_->publish(output);
}

}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)
