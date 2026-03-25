#include "scenarios.hpp"

#include <stdexcept>

namespace {
    constexpr double kSegmentDurationSec = 1.0;
    constexpr double kTotalDurationSec = 4.0;
}

ScenarioConfig makeScenarioConfig(ScenarioMode mode, const RobotModel& robot) {
    ScenarioConfig config{};

    switch (mode) {
        case ScenarioMode::FREE_SPACE_MOTION:
            config.initial_xy = Eigen::Vector2d(1.0, 0.0);
            config.initial_xy_d = config.initial_xy;
            config.initial_q = robot.inverseKinematics(config.initial_xy(0), config.initial_xy(1));
            config.initial_qdot = Eigen::Vector2d::Zero();
            config.initial_q_d = robot.inverseKinematics(config.initial_xy_d(0), config.initial_xy_d(1));
            config.initial_qdot_d = Eigen::Vector2d::Zero();
            config.total_time_sec = kTotalDurationSec;
            config.desired_xy_trajectory = &runFreeSpaceMotion;
            return config;
        case ScenarioMode::STIFF_ENVIRONMENT:
            config.initial_xy = Eigen::Vector2d(0.5, 0.0);
            config.initial_xy_d = config.initial_xy;
            config.initial_q = robot.inverseKinematics(config.initial_xy(0), config.initial_xy(1));
            config.initial_qdot = Eigen::Vector2d::Zero();
            config.initial_q_d = robot.inverseKinematics(config.initial_xy_d(0), config.initial_xy_d(1));
            config.initial_qdot_d = Eigen::Vector2d::Zero();
            config.total_time_sec = kTotalDurationSec;
            config.desired_xy_trajectory = &runStiffEnvironment;
            return config;
        case ScenarioMode::EXTERNAL_DISTURBANCE:
            config.initial_xy = Eigen::Vector2d(1.0, 0.0);
            config.initial_xy_d = config.initial_xy;
            config.initial_q = robot.inverseKinematics(config.initial_xy(0), config.initial_xy(1));
            config.initial_qdot = Eigen::Vector2d::Zero();
            config.initial_q_d = robot.inverseKinematics(config.initial_xy_d(0), config.initial_xy_d(1));
            config.initial_qdot_d = Eigen::Vector2d::Zero();
            config.total_time_sec = kTotalDurationSec;
            config.desired_xy_trajectory = &runExternalDisturbance;
            return config;
        default:
            throw std::invalid_argument("Scenario mode is not implemented.");
    }
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

Eigen::Vector2d runStiffEnvironment(double time_sec) {
    if (time_sec <= 0.0 || time_sec >= kTotalDurationSec) {
        return Eigen::Vector2d(0.5, 0.0);
    }

    if (time_sec < 2.0) {
        const double alpha = time_sec / 2.0;
        return Eigen::Vector2d(0.5 + 1.0 * alpha, 0.0);
    }

    // Return to the initial position
    const double alpha = (time_sec - 2.0) / 2.0;
    return Eigen::Vector2d(1.5 - 1.0 * alpha, 0.0);
}

Eigen::Vector2d runExternalDisturbance(double time_sec) {
    // The end-effector is supposed to stay at (1.0, 0.0)
    return Eigen::Vector2d(1.0, 0.0);
}