#ifndef FAKE_VEL_TRANSFORM__VELOCITY_WINDOW_HPP_
#define FAKE_VEL_TRANSFORM__VELOCITY_WINDOW_HPP_

#include <cmath>
#include <deque>
#include <stdexcept>

namespace fake_vel_transform
{
// Feed vectors in one fixed frame. A zero window preserves the latest value.
class VelocityWindow
{
public:
  struct Velocity {double x; double y;};

  explicit VelocityWindow(double duration) : duration_(duration)
  {
    if (!std::isfinite(duration) || duration < 0.0) {
      throw std::invalid_argument("velocity window must be finite and nonnegative");
    }
  }

  bool update(double stamp, double x, double y, Velocity & result)
  {
    if (!std::isfinite(stamp) || !std::isfinite(x) || !std::isfinite(y) ||
      (!samples_.empty() && stamp <= samples_.back().stamp))
    {
      return false;
    }
    while (!samples_.empty() && stamp - samples_.front().stamp > duration_) {
      samples_.pop_front();
    }
    samples_.push_back({stamp, x, y});
    result = {0.0, 0.0};
    for (const auto & sample : samples_) {
      result.x += sample.x;
      result.y += sample.y;
    }
    result.x /= samples_.size();
    result.y /= samples_.size();
    return true;
  }

  void reset() {samples_.clear();}

private:
  struct Sample {double stamp; double x; double y;};
  double duration_;
  std::deque<Sample> samples_;
};
}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__VELOCITY_WINDOW_HPP_
