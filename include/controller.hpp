#pragma once
#include "simulator.hpp"

class Controller {
public:
    Controller(double Kp, double Kd, double tau_max);
    void computeTorque(const RobotModel& robot, const RobotState& state, const DesiredRobotState& desired_state) const;

private:
    double Kp_;
    double Kd_;
    double tau_max_;
};