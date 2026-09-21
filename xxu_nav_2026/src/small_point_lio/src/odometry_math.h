#pragma once

#include <Eigen/Geometry>

namespace small_point_lio {

// All vectors are in odom coordinates; displacement points from lidar to base.
inline Eigen::Vector3d baseVelocityFromLidar(
        const Eigen::Vector3d &lidar_velocity,
        const Eigen::Vector3d &angular_velocity,
        const Eigen::Vector3d &lidar_to_base)
{
    return lidar_velocity + angular_velocity.cross(lidar_to_base);
}

}  // namespace small_point_lio
