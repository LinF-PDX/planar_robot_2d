#pragma once
#include "robot_model.hpp"

enum class ScenarioMode {
    UNDEFINED = 0,
    FREE_SPACE_MOTION = 1,
    STIFF_ENVIRONMENT = 2,
    EXTERNAL_DISTURBANCE = 3
};

struct ScenarioConfig {
    Eigen::Vector2d initial_q;
    Eigen::Vector2d initial_qdot;
    Eigen::Vector2d initial_q_d;
    Eigen::Vector2d initial_qdot_d;
    Eigen::Vector2d initial_xy;
    Eigen::Vector2d initial_xy_d;
    double total_time_sec;
    Eigen::Vector2d (*desired_xy_trajectory)(double time_sec);
};

ScenarioConfig makeScenarioConfig(ScenarioMode mode, const RobotModel& robot);

Eigen::Vector2d runFreeSpaceMotion(double time_sec);
