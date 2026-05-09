#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace xxu_pointcloud_processing
{

struct RotationMatrix
{
  double m[3][3];
};

struct PointFields
{
  uint32_t x_offset;
  uint32_t y_offset;
  uint32_t z_offset;
};

class PointCloudProcessor : public rclcpp::Node
{
public:
  PointCloudProcessor()
  : Node("pointcloud_processor"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_frame_ = this->declare_parameter<std::string>("input_frame", "radar_link");
    output_frame_ = this->declare_parameter<std::string>("output_frame", "base_footprint");
    project_to_2d_ = this->declare_parameter<bool>("project_to_2d", true);
    z_value_ = this->declare_parameter<double>("z_value", 0.0);
    min_height_ = this->declare_parameter<double>("min_height", 0.05);
    max_height_ = this->declare_parameter<double>("max_height", 0.5);
    min_range_ = this->declare_parameter<double>("min_range", 0.05);
    max_range_ = this->declare_parameter<double>("max_range", 40.0);
    min_x_ = this->declare_parameter<double>("min_x", -40.0);
    max_x_ = this->declare_parameter<double>("max_x", 40.0);
    min_y_ = this->declare_parameter<double>("min_y", -40.0);
    max_y_ = this->declare_parameter<double>("max_y", 40.0);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_in",
      rclcpp::SensorDataQoS(),
      std::bind(&PointCloudProcessor::pointCloudCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "points_out",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    RCLCPP_INFO(
      this->get_logger(),
      "Point cloud processor: input_frame=%s, output_frame=%s, height=[%.3f, %.3f], "
      "range=[%.3f, %.3f], crop_x=[%.3f, %.3f], crop_y=[%.3f, %.3f], project_to_2d=%s",
      input_frame_.c_str(), output_frame_.c_str(), min_height_, max_height_,
      min_range_, max_range_, min_x_, max_x_, min_y_, max_y_,
      project_to_2d_ ? "true" : "false");
  }

private:
  static bool readFloat32(
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

    const bool host_bigendian = isHostBigendian();
    if (host_bigendian != is_bigendian) {
      std::reverse(bytes, bytes + sizeof(float));
    }

    std::memcpy(&value, bytes, sizeof(float));
    return true;
  }

  static bool writeFloat32(
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

    const bool host_bigendian = isHostBigendian();
    if (host_bigendian != is_bigendian) {
      std::reverse(bytes, bytes + sizeof(float));
    }

    std::memcpy(data.data() + offset, bytes, sizeof(float));
    return true;
  }

  static bool isHostBigendian()
  {
    const uint16_t value = 0x0102;
    return *reinterpret_cast<const uint8_t *>(&value) == 0x01;
  }

  static bool findFields(const sensor_msgs::msg::PointCloud2 & msg, PointFields & fields)
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

  static RotationMatrix makeRotation(
    const geometry_msgs::msg::TransformStamped & transform)
  {
    auto q = transform.transform.rotation;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (norm <= std::numeric_limits<double>::epsilon()) {
      q.x = 0.0;
      q.y = 0.0;
      q.z = 0.0;
      q.w = 1.0;
    } else {
      q.x /= norm;
      q.y /= norm;
      q.z /= norm;
      q.w /= norm;
    }

    RotationMatrix rotation{};
    rotation.m[0][0] = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    rotation.m[0][1] = 2.0 * (q.x * q.y - q.z * q.w);
    rotation.m[0][2] = 2.0 * (q.x * q.z + q.y * q.w);
    rotation.m[1][0] = 2.0 * (q.x * q.y + q.z * q.w);
    rotation.m[1][1] = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
    rotation.m[1][2] = 2.0 * (q.y * q.z - q.x * q.w);
    rotation.m[2][0] = 2.0 * (q.x * q.z - q.y * q.w);
    rotation.m[2][1] = 2.0 * (q.y * q.z + q.x * q.w);
    rotation.m[2][2] = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    return rotation;
  }

  bool passCropAndRecognition(const double x, const double y, const double z) const
  {
    if (z < min_height_ || z > max_height_) {
      return false;
    }

    if (x < min_x_ || x > max_x_ || y < min_y_ || y > max_y_) {
      return false;
    }

    const double range = std::hypot(x, y);
    return range >= min_range_ && range <= max_range_;
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 input = *msg;
    input.header.frame_id = input_frame_;

    PointFields fields{};
    if (!findFields(input, fields)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "PointCloud2 x/y/z fields must exist and be FLOAT32");
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        output_frame_,
        input.header.frame_id,
        input.header.stamp,
        rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Cannot transform point cloud to %s: %s", output_frame_.c_str(), ex.what());
      return;
    }

    const RotationMatrix rotation = makeRotation(transform);
    const auto & translation = transform.transform.translation;

    sensor_msgs::msg::PointCloud2 output;
    output.header = input.header;
    output.header.frame_id = output_frame_;
    output.height = 1;
    output.fields = input.fields;
    output.is_bigendian = input.is_bigendian;
    output.point_step = input.point_step;
    output.is_dense = false;
    output.data.reserve(input.data.size());

    for (uint32_t row = 0; row < input.height; ++row) {
      const size_t row_start = static_cast<size_t>(row) * input.row_step;
      for (uint32_t column = 0; column < input.width; ++column) {
        const size_t point_start = row_start + static_cast<size_t>(column) * input.point_step;
        if (point_start + input.point_step > input.data.size()) {
          continue;
        }

        float x = 0.0F;
        float y = 0.0F;
        float z = 0.0F;
        if (!readFloat32(input.data, point_start + fields.x_offset, input.is_bigendian, x) ||
          !readFloat32(input.data, point_start + fields.y_offset, input.is_bigendian, y) ||
          !readFloat32(input.data, point_start + fields.z_offset, input.is_bigendian, z))
        {
          continue;
        }

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }

        const double out_x =
          rotation.m[0][0] * x + rotation.m[0][1] * y + rotation.m[0][2] * z + translation.x;
        const double out_y =
          rotation.m[1][0] * x + rotation.m[1][1] * y + rotation.m[1][2] * z + translation.y;
        const double out_z =
          rotation.m[2][0] * x + rotation.m[2][1] * y + rotation.m[2][2] * z + translation.z;

        if (!passCropAndRecognition(out_x, out_y, out_z)) {
          continue;
        }

        const size_t output_start = output.data.size();
        output.data.insert(
          output.data.end(),
          input.data.begin() + static_cast<std::vector<uint8_t>::difference_type>(point_start),
          input.data.begin() + static_cast<std::vector<uint8_t>::difference_type>(
            point_start + input.point_step));

        writeFloat32(output.data, output_start + fields.x_offset, input.is_bigendian, out_x);
        writeFloat32(output.data, output_start + fields.y_offset, input.is_bigendian, out_y);
        writeFloat32(
          output.data,
          output_start + fields.z_offset,
          input.is_bigendian,
          project_to_2d_ ? z_value_ : out_z);
      }
    }

    output.width = output.data.size() / output.point_step;
    output.row_step = output.data.size();
    pub_->publish(output);
  }

  std::string input_frame_;
  std::string output_frame_;
  bool project_to_2d_;
  double z_value_;
  double min_height_;
  double max_height_;
  double min_range_;
  double max_range_;
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace xxu_pointcloud_processing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xxu_pointcloud_processing::PointCloudProcessor>());
  rclcpp::shutdown();
  return 0;
}
