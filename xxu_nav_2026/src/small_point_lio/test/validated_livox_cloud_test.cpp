#include "lidar_adapter/validated_livox_cloud.h"
#include <cassert>
#include <limits>

int main() {
    using namespace small_point_lio;
    sensor_msgs::msg::PointCloud2 msg;
    msg.width = 1; msg.height = 2; msg.point_step = 24; msg.row_step = 32;
    msg.data.resize(64);
    for (const auto &name : {"x", "y", "z", "tag", "timestamp"}) {
        sensor_msgs::msg::PointField f; f.name = name; f.count = 1;
        f.offset = f.name == "x" ? 0 : f.name == "y" ? 4 : f.name == "z" ? 8 : f.name == "tag" ? 12 : 16;
        f.datatype = f.name == "timestamp" ? 8 : f.name == "tag" ? 2 : 7;
        msg.fields.push_back(f);
    }
    std::array<uint32_t, 5> offsets{};
    assert(validateLivoxCloud(msg, offsets)); // Organized cloud with padding.
    auto broken = msg; broken.data.resize(24);
    assert(!validateLivoxCloud(broken, offsets));
    broken = msg; broken.fields.pop_back();
    assert(!validateLivoxCloud(broken, offsets));
    broken = msg; broken.fields.back().datatype = 7;
    assert(!validateLivoxCloud(broken, offsets));
    broken = msg; broken.fields.back().offset = std::numeric_limits<uint32_t>::max();
    assert(!validateLivoxCloud(broken, offsets));
    broken = msg; broken.point_step = 0;
    assert(!validateLivoxCloud(broken, offsets));
    broken = msg; broken.width = std::numeric_limits<uint32_t>::max();
    assert(!validateLivoxCloud(broken, offsets));
    const uint8_t big[] = {0x3f, 0x80, 0, 0};
    const uint8_t little[] = {0, 0, 0x80, 0x3f};
    assert(readLivoxValue<float>(big, true) == 1.0f);
    assert(readLivoxValue<float>(little, false) == 1.0f);
}
