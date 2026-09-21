#include "odometry_math.h"

#include <cassert>
#include <cmath>
#include <limits>

int main()
{
    // Volatile inputs exercise runtime checks under the production compiler flags.
    volatile double nan = std::numeric_limits<double>::quiet_NaN();
    volatile double inf = std::numeric_limits<double>::infinity();
    assert(!std::isfinite(nan));
    assert(!std::isfinite(inf));
    assert(!std::isfinite(-inf));

    using Eigen::Vector3d;
    using small_point_lio::baseVelocityFromLidar;
    // Base rotates in place, lidar is 1 m ahead: lidar moves left at 2 m/s.
    assert(baseVelocityFromLidar({0, 2, 0}, {0, 0, 2}, {-1, 0, 0}).isZero());
    assert(baseVelocityFromLidar({3, 2, 0}, {0, 0, 2}, {-1, 0, 0})
            .isApprox(Vector3d(3, 0, 0)));
    assert(baseVelocityFromLidar({3, 4, 5}, Vector3d::Zero(), {1, 2, 3})
            .isApprox(Vector3d(3, 4, 5)));
    // Tilted rotation axis and a vertical lever arm.
    assert(baseVelocityFromLidar({0, -6, 0}, {2, 0, 0}, {0, 0, -3}).isZero());
}
