#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>

namespace small_point_lio {

// A sensor-time discontinuity cannot be repaired by repeatedly propagating
// the last acceleration. Keep the fault latched until estimator reset/restart.
class ImuTimeGuard {
public:
    enum class Result { Accepted, Stale, Fault };

    explicit ImuTimeGuard(double max_gap) : max_gap_(max_gap) {
        if (!std::isfinite(max_gap) || max_gap <= 0.0) {
            throw std::invalid_argument("max_imu_sample_gap must be finite and positive");
        }
    }

    void reset(double timestamp = std::numeric_limits<double>::quiet_NaN()) {
        last_timestamp_ = timestamp;
        faulted_ = false;
        gap_ = 0.0;
    }

    Result observe(double timestamp) {
        if (faulted_) return Result::Fault;
        if (!std::isfinite(timestamp)) return Result::Stale;
        if (std::isfinite(last_timestamp_)) {
            if (timestamp <= last_timestamp_) return Result::Stale;
            const double dt = timestamp - last_timestamp_;
            if (dt > max_gap_ + 1e-9) {
                faulted_ = true;
                gap_ = dt;
                return Result::Fault;
            }
        }
        last_timestamp_ = timestamp;
        return Result::Accepted;
    }

    [[nodiscard]] bool faulted() const { return faulted_; }
    [[nodiscard]] double gap() const { return gap_; }
    [[nodiscard]] double last_timestamp() const { return last_timestamp_; }

private:
    double max_gap_;
    double last_timestamp_ = std::numeric_limits<double>::quiet_NaN();
    double gap_ = 0.0;
    bool faulted_ = false;
};

}  // namespace small_point_lio
