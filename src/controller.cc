#include "controller.hpp"

Controller::Controller(double Kp, double Kd, double tau_max)
    : Kp_(Kp), Kd_(Kd), tau_max_(tau_max) {}

Eigen::Vector2d Controller::computeTorque(const RobotModel& robot, const RobotState& state, const DesiredRobotState& desired_state) const {
    // Compute the control torque using a PD control law with gravity compensation
    Eigen::Vector2d tau;
    Eigen::Vector2d e = desired_state.q_desired - state.q; // Position error
    Eigen::Vector2d edot = desired_state.qdot_desired - state.qdot; // Velocity error

    // Joint-space impedance control
    tau = Kp_ * e + Kd_ * edot + robot.getGravityVector(state.q);

    // Saturate the torque to the maximum allowed value
    tau = tau.cwiseMin(Eigen::Vector2d::Constant(tau_max_)).cwiseMax(Eigen::Vector2d::Constant(-tau_max_));

    return tau;
}