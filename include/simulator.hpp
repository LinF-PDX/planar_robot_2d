#pragma once
#include "robot_model.hpp"

// Robot state structure
struct RobotState {
    Eigen::Vector2d q = Eigen::Vector2d::Zero(); // Joint angles
    Eigen::Vector2d qdot = Eigen::Vector2d::Zero(); // Joint velocities
    double time = 0.0; // Current time
};

class Simulator {
public:
    explicit Simulator(double dt);
    void stepSimulation(const RobotModel& robot, const Eigen::Vector2d& tau, RobotState& state) const;
    double getTimeStep() const; 

private:
    double dt_; // Time step for the simulation
};