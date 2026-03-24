#pragma once
#include "robot_model.hpp"

// Robot state structure
struct RobotState {
    Eigen::Vector2d q = Eigen::Vector2d::Zero(); // Joint angles
    Eigen::Vector2d qdot = Eigen::Vector2d::Zero(); // Joint velocities
    double time = 0.0; // Current time
};

struct DesiredRobotState {
    Eigen::Vector2d q_d = Eigen::Vector2d::Zero(); // Desired joint angles
    Eigen::Vector2d qdot_d = Eigen::Vector2d::Zero(); // Desired joint velocities
    Eigen::Vector2d q_d_prev = Eigen::Vector2d::Zero(); // Previous desired joint angles (for numerical differentiation)
};

class Simulator {
public:
    explicit Simulator(double dt);
    void stepSimulation(const RobotModel& robot, const Eigen::Vector2d& tau, RobotState& state) const;

private:
    double dt_; // Time step for the simulation
};