#include "simulator.hpp"

Simulator::Simulator(double dt) : dt_(dt) {}

void Simulator::stepSimulation(const RobotModel& robot, const Eigen::Vector2d& tau, RobotState& state) const {
/*
    // Semi-implicit Euler integration
    Eigen::Vector2d qddot = robot.forwardDynamics(state.q, state.qdot, tau);
    state.qdot += qddot * dt_;
    state.q += state.qdot * dt_;
    state.time += dt_;
*/

    // Runge-Kutta 4 integration
    const double h = dt_;

    // k1
    Eigen::Vector2d k1_q = state.qdot;
    Eigen::Vector2d k1_qdot = robot.forwardDynamics(state.q, state.qdot, tau);

    // k2
    Eigen::Vector2d q_k2 = state.q + 0.5 * h * k1_q;
    Eigen::Vector2d qdot_k2 = state.qdot + 0.5 * h * k1_qdot;
    Eigen::Vector2d k2_q = qdot_k2;
    Eigen::Vector2d k2_qdot = robot.forwardDynamics(q_k2, qdot_k2, tau);

    // k3
    Eigen::Vector2d q_k3 = state.q + 0.5 * h * k2_q;
    Eigen::Vector2d qdot_k3 = state.qdot + 0.5 * h * k2_qdot;
    Eigen::Vector2d k3_q = qdot_k3;
    Eigen::Vector2d k3_qdot = robot.forwardDynamics(q_k3, qdot_k3, tau);

    // k4
    Eigen::Vector2d q_k4 = state.q + h * k3_q;
    Eigen::Vector2d qdot_k4 = state.qdot + h * k3_qdot;
    Eigen::Vector2d k4_q = qdot_k4;
    Eigen::Vector2d k4_qdot = robot.forwardDynamics(q_k4, qdot_k4, tau);

    // Final RK4 update
    state.q += (h / 6.0) * (k1_q + 2.0 * k2_q + 2.0 * k3_q + k4_q);
    state.qdot += (h / 6.0) * (k1_qdot + 2.0 * k2_qdot + 2.0 * k3_qdot + k4_qdot);

    // Update time
    state.time += h;
}

Eigen::Vector2d Simulator::wallContactForce(const Eigen::Vector2d& xy) const {
    // Simple wall contact model, assuming the wall is at x = 1.2
    const double wall_x = 1.2;
    const double wall_stiffness = 10000.0; // Contact stiffness
    Eigen::Vector2d contactForce = Eigen::Vector2d::Zero();
    if (xy(0) > wall_x) {
        contactForce(0) = -wall_stiffness * (xy(0) - wall_x);
    }
    return contactForce;
}

Eigen::Vector2d Simulator::externalDisturbanceForce(double time_sec) const {
    // Apply impluse force in -y direction at t = 1.0s for 0.5s
    Eigen::Vector2d disturbanceForce = Eigen::Vector2d::Zero();
    if (time_sec >= 1.0 && time_sec < 1.5) {
        disturbanceForce(1) = -20.0; // Impulse force in -y direction
    } else {
        disturbanceForce.setZero();
    }
    return disturbanceForce;
}
