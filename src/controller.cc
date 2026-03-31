#include "controller.hpp"

Controller::Controller(double Kp, double Kd, double tau_max)
    : Kp_(Kp), Kd_(Kd), tau_max_(tau_max) {}

Eigen::Vector2d Controller::computeTorque(const RobotModel& robot, const RobotState& state, const DesiredRobotState& desired_state, const double dt) const {
    // Compute the control torque using a PD control law with gravity compensation
    Eigen::Vector2d tau;
    Eigen::Vector2d e = desired_state.q_d - state.q; // Position error
    Eigen::Vector2d edot = desired_state.qdot_d - state.qdot; // Velocity error
    Eigen::Vector2d qddot_d = (desired_state.qdot_d - desired_state.qdot_d_prev) / dt;

    Eigen::Matrix2d M = robot.getMassMatrix(state.q);
    Eigen::Vector2d C = robot.getCoriolisVector(state.q, state.qdot);
    Eigen::Vector2d G = robot.getGravityVector(state.q);

    // Joint-space impedance control
    tau = M * qddot_d + C + G + Kp_ * e + Kd_ * edot;
    // tau = Kp_ * e + Kd_ * edot + G; // PD control with gravity compensation only, ignoring desired acceleration for simplicity

    // Saturate the torque to the maximum allowed value
    tau = tau.cwiseMin(Eigen::Vector2d::Constant(tau_max_)).cwiseMax(Eigen::Vector2d::Constant(-tau_max_));

    return tau;
}