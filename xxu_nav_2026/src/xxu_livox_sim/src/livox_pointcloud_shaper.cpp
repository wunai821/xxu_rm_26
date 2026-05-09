#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace xxu_livox_sim
{

struct FieldInfo
{
  uint32_t offset = 0;
  uint8_t datatype = 0;
  bool exists = false;
};

class LivoxPointCloudShaper : public rclcpp::Node
{
public:
  LivoxPointCloudShaper()
  : Node("livox_pointcloud_shaper")
  {
    target_frame_ = this->declare_parameter<std::string>("target_frame", "radar_link");
    scan_period_ = this->declare_parameter<double>("scan_period", 0.1);
    max_points_ = this->declare_parameter<int>("max_points", 12000);
    scan_mode_csv_ = this->declare_parameter<std::string>("scan_mode_csv", "");

    if (!scan_mode_csv_.empty()) {
      std::ifstream csv(scan_mode_csv_);
      RCLCPP_INFO(
        this->get_logger(), "Livox scan mode CSV %s: %s",
        scan_mode_csv_.c_str(), csv.good() ? "available" : "not found");
    }

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_in",
      rclcpp::SensorDataQoS(),
      std::bind(&LivoxPointCloudShaper::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "points_out",
      rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      this->get_logger(),
      "Livox point cloud shaper: target_frame=%s, scan_period=%.3f, max_points=%d",
      target_frame_.c_str(), scan_period_, max_points_);
  }

private:
  static FieldInfo findField(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
  {
    for (const auto & field : msg.fields) {
      if (field.name == name && field.count == 1) {
        return FieldInfo{field.offset, field.datatype, true};
      }
    }
    return {};
  }

  static bool hostIsBigendian()
  {
    const uint16_t value = 0x0102;
    return *reinterpret_cast<const uint8_t *>(&value) == 0x01;
  }

  template<typename T>
  static bool readValue(
    const sensor_msgs::msg::PointCloud2 & msg,
    const size_t offset,
    T & value)
  {
    if (offset + sizeof(T) > msg.data.size()) {
      return false;
    }

    uint8_t bytes[sizeof(T)];
    std::memcpy(bytes, msg.data.data() + offset, sizeof(T));
    if (hostIsBigendian() != msg.is_bigendian) {
      std::reverse(bytes, bytes + sizeof(T));
    }
    std::memcpy(&value, bytes, sizeof(T));
    return true;
  }

  template<typename T>
  static void writeValue(std::vector<uint8_t> & data, const size_t offset, const T value)
  {
    std::memcpy(data.data() + offset, &value, sizeof(T));
  }

  static bool readFloat32(
    const sensor_msgs::msg::PointCloud2 & msg,
    const size_t point_offset,
    const FieldInfo & field,
    float & value)
  {
    if (!field.exists || field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
      return false;
    }
    if (!readValue(msg, point_offset + field.offset, value)) {
      return false;
    }
    return std::isfinite(value);
  }

  static bool readUint16(
    const sensor_msgs::msg::PointCloud2 & msg,
    const size_t point_offset,
    const FieldInfo & field,
    uint16_t & value)
  {
    if (!field.exists || field.datatype != sensor_msgs::msg::PointField::UINT16) {
      return false;
    }
    return readValue(msg, point_offset + field.offset, value);
  }

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto x_field = findField(*msg, "x");
    const auto y_field = findField(*msg, "y");
    const auto z_field = findField(*msg, "z");
    const auto intensity_field = findField(*msg, "intensity");
    const auto ring_field = findField(*msg, "ring");

    if (!x_field.exists || !y_field.exists || !z_field.exists) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "PointCloud2 x/y/z FLOAT32 fields are required");
      return;
    }

    const size_t input_size = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    if (input_size == 0 || msg->point_step == 0) {
      return;
    }

    const size_t max_points = max_points_ > 0 ? static_cast<size_t>(max_points_) : input_size;
    const size_t stride = std::max<size_t>(1, static_cast<size_t>(std::ceil(
      static_cast<double>(input_size) / static_cast<double>(max_points))));
    const size_t output_size = (input_size + stride - 1) / stride;

    sensor_msgs::msg::PointCloud2 output;
    output.header = msg->header;
    output.header.frame_id = target_frame_;
    output.height = 1;
    output.width = static_cast<uint32_t>(output_size);
    output.is_bigendian = false;
    output.is_dense = false;
    output.point_step = 32;
    output.row_step = output.point_step * output.width;
    output.fields = makeLivoxFields();
    output.data.resize(static_cast<size_t>(output.row_step));

    const double stamp_ns =
      static_cast<double>(msg->header.stamp.sec) * 1.0e9 +
      static_cast<double>(msg->header.stamp.nanosec);
    const double scan_period_ns = scan_period_ * 1.0e9;

    size_t out_index = 0;
    for (size_t in_index = 0; in_index < input_size && out_index < output_size; in_index += stride) {
      const size_t input_offset = in_index * msg->point_step;
      float x = 0.0F;
      float y = 0.0F;
      float z = 0.0F;
      if (!readFloat32(*msg, input_offset, x_field, x) ||
          !readFloat32(*msg, input_offset, y_field, y) ||
          !readFloat32(*msg, input_offset, z_field, z)) {
        continue;
      }

      float intensity = 0.0F;
      (void)readFloat32(*msg, input_offset, intensity_field, intensity);

      uint16_t ring = 0;
      (void)readUint16(*msg, input_offset, ring_field, ring);

      const double ratio = output_size > 1 ?
        static_cast<double>(out_index) / static_cast<double>(output_size - 1) : 0.0;
      const double timestamp_ns = stamp_ns + scan_period_ns * ratio;
      const size_t output_offset = out_index * output.point_step;

      writeValue(output.data, output_offset + 0, x);
      writeValue(output.data, output_offset + 4, y);
      writeValue(output.data, output_offset + 8, z);
      writeValue(output.data, output_offset + 12, intensity);
      writeValue<uint8_t>(output.data, output_offset + 16, 0U);
      writeValue<uint8_t>(output.data, output_offset + 17, static_cast<uint8_t>(ring & 0xFFU));
      writeValue(output.data, output_offset + 24, timestamp_ns);
      ++out_index;
    }

    output.width = static_cast<uint32_t>(out_index);
    output.row_step = output.point_step * output.width;
    output.data.resize(static_cast<size_t>(output.row_step));
    pub_->publish(output);
  }

  static std::vector<sensor_msgs::msg::PointField> makeLivoxFields()
  {
    std::vector<sensor_msgs::msg::PointField> fields;
    fields.reserve(7);

    auto add_field = [&fields](const std::string & name, const uint32_t offset, const uint8_t type) {
      sensor_msgs::msg::PointField field;
      field.name = name;
      field.offset = offset;
      field.datatype = type;
      field.count = 1;
      fields.push_back(field);
    };

    add_field("x", 0, sensor_msgs::msg::PointField::FLOAT32);
    add_field("y", 4, sensor_msgs::msg::PointField::FLOAT32);
    add_field("z", 8, sensor_msgs::msg::PointField::FLOAT32);
    add_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
    add_field("tag", 16, sensor_msgs::msg::PointField::UINT8);
    add_field("line", 17, sensor_msgs::msg::PointField::UINT8);
    add_field("timestamp", 24, sensor_msgs::msg::PointField::FLOAT64);
    return fields;
  }

  std::string target_frame_;
  double scan_period_;
  int max_points_;
  std::string scan_mode_csv_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace xxu_livox_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xxu_livox_sim::LivoxPointCloudShaper>());
  rclcpp::shutdown();
  return 0;
}
