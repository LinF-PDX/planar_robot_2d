#include "simulator.hpp"

Simulator::Simulator(double dt) : dt_(dt) {}

void Simulator::stepSimulation(const RobotModel& robot, const Eigen::Vector2d& tau, RobotState& state) const {
    // Compute joint accelerations using forward dynamics
    Eigen::Vector2d qddot = robot.forwardDynamics(state.q, state.qdot, tau);

    // Update joint velocities and angles using Euler integration
    state.qdot += qddot * dt_;
    state.q += state.qdot * dt_;

    // Update time
    state.time += dt_;
}

double Simulator::getTimeStep() const {
    return dt_;
}