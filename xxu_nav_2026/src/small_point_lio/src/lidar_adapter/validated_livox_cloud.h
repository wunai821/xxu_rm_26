#pragma once

#include <array>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace small_point_lio {

// Validate before allocating from metadata or accessing any payload bytes.
inline bool validateLivoxCloud(
    const sensor_msgs::msg::PointCloud2 &msg, std::array<uint32_t, 5> &offsets) {
    const std::array<const char *, 5> names{"x", "y", "z", "tag", "timestamp"};
    const std::array<uint8_t, 5> types{7, 7, 7, 2, 8};
    const std::array<uint32_t, 5> sizes{4, 4, 4, 1, 8};
    if (msg.point_step == 0 ||
        uint64_t(msg.width) * msg.point_step > msg.row_step ||
        uint64_t(msg.row_step) * msg.height != msg.data.size()) {
        return false;
    }
    for (size_t i = 0; i < names.size(); ++i) {
        bool found = false;
        for (const auto &field : msg.fields) {
            if (field.name != names[i]) continue;
            if (found || field.count != 1 || field.datatype != types[i] ||
                uint64_t(field.offset) + sizes[i] > msg.point_step) return false;
            offsets[i] = field.offset;
            found = true;
        }
        if (!found) return false;
    }
    return true;
}

template<class T>
inline T readLivoxValue(const uint8_t *data, bool bigendian) {
    std::array<uint8_t, sizeof(T)> bytes;
    std::memcpy(bytes.data(), data, sizeof(T));
    const uint16_t probe = 0x0102;
    const bool host_bigendian = *reinterpret_cast<const uint8_t *>(&probe) == 1;
    if (bigendian != host_bigendian) std::reverse(bytes.begin(), bytes.end());
    T value;
    std::memcpy(&value, bytes.data(), sizeof(T));
    return value;
}
}  // namespace small_point_lio
