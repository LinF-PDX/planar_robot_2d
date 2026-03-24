#include "scenarios.hpp"

namespace {
constexpr double kSegmentDurationSec = 1.0;
constexpr double kTotalDurationSec = 4.0;
}

Eigen::Vector2d runFreeSpaceMotion(double time_sec) {
    if (time_sec <= 0.0 || time_sec >= kTotalDurationSec) {
        return Eigen::Vector2d(1.0, 0.0);
    }

    if (time_sec < 1.0 * kSegmentDurationSec) {
        const double alpha = time_sec / kSegmentDurationSec;
        return Eigen::Vector2d(1.0 + 0.5 * alpha, 0.0);
    }

    if (time_sec < 2.0 * kSegmentDurationSec) {
        const double alpha = (time_sec - 1.0 * kSegmentDurationSec) / kSegmentDurationSec;
        return Eigen::Vector2d(1.5, 0.5 * alpha);
    }

    if (time_sec < 3.0 * kSegmentDurationSec) {
        const double alpha = (time_sec - 2.0 * kSegmentDurationSec) / kSegmentDurationSec;
        return Eigen::Vector2d(1.5 - 0.5 * alpha, 0.5);
    }

    const double alpha = (time_sec - 3.0 * kSegmentDurationSec) / kSegmentDurationSec;
    return Eigen::Vector2d(1.0, 0.5 - 0.5 * alpha);
}
