#include <iostream>
#include "logger.hpp"
#include "simulator.hpp"
#include "controller.hpp"
#include "scenarios.hpp"

namespace {
    constexpr ScenarioMode kScenarioMode = ScenarioMode::FREE_SPACE_MOTION;
    constexpr double kSimulationTotalTimeSec = 5.0;
    constexpr double kSimulationTimeStep = 1e-4;
    constexpr double kSimulationLogInterval = 1e-2;
    constexpr int KSimulationTotalSteps = static_cast<int>(kSimulationTotalTimeSec / kSimulationTimeStep);

    constexpr double kRobotLink1Length = 1.0;
    constexpr double kRobotLink2Length = 1.0;
    constexpr double kRobotLink1Mass = 1.0;
    constexpr double kRobotLink2Mass = 1.0;

    constexpr double kKp = 50.0;
    constexpr double kKd = 10.0;
    constexpr double kTauMax = 50.0;
}

int main() {
    RobotModel robot(kRobotLink1Length, kRobotLink2Length, kRobotLink1Mass, kRobotLink2Mass);
    Simulator sim(kSimulationTimeStep);
    Controller controller(kKp, kKd, kTauMax);
    const ScenarioConfig scenario_config = makeScenarioConfig(kScenarioMode, robot);

    RobotState state;
    state.q = scenario_config.initial_q;
    state.qdot = scenario_config.initial_qdot;

    DesiredRobotState desired_state;
    desired_state.q_d = scenario_config.initial_q_d;
    desired_state.qdot_d = scenario_config.initial_qdot_d;
    desired_state.q_d_prev = scenario_config.initial_q_d;

    // Convert radians to degrees for logging
    double q1_deg = state.q(0) * 180 / M_PI;
    double q2_deg = state.q(1) * 180 / M_PI;
    double q1dot_deg = state.qdot(0) * 180 / M_PI;
    double q2dot_deg = state.qdot(1) * 180 / M_PI;
    double q1_d_deg = desired_state.q_d(0) * 180 / M_PI;
    double q2_d_deg = desired_state.q_d(1) * 180 / M_PI;

    Eigen::Vector2d tau;
    tau << 0.0, 0.0;

    Eigen::Vector2d tau_external;
    tau_external << 0.0, 0.0;

    // Actual end-effector position
    Eigen::Vector2d xy = scenario_config.initial_xy;

    // Desired end-effector position
    Eigen::Vector2d xy_d = scenario_config.initial_xy_d;

    // Initialize logger
    SignalLogger logger("data", {"time", "q1", "q2", "q1_d", "q2_d", "x", "y", "tau1", "tau2","xy_d_x", "xy_d_y"});

    // Write first log entry
    double next_log_time = 0.0;
    logger.writeRow({state.time, q1_deg, q2_deg, q1_d_deg, q2_d_deg, xy(0), xy(1), tau(0), tau(1), xy_d(0), xy_d(1)});
    next_log_time += kSimulationLogInterval;

    // Start simulation loop
    for (int i = 0; i < KSimulationTotalSteps; ++i) {
        // Update desired end-effector position based on the scenario
        xy_d = scenario_config.desired_xy_trajectory(state.time);

        // Get desired joint angles and velocities from inverse kinematics
        desired_state.q_d = robot.inverseKinematics(xy_d(0), xy_d(1));
        desired_state.qdot_d = (desired_state.q_d - desired_state.q_d_prev) / kSimulationTimeStep; // Numerical differentiation
        desired_state.q_d_prev = desired_state.q_d;

        // Compute control torque
        tau = controller.computeTorque(robot, state, desired_state);

        // Compute external forces based on the scenario and convert them to joint torques
        switch (ScenarioMode(kScenarioMode)) {
            case ScenarioMode::FREE_SPACE_MOTION:
                tau_external.setZero();
                break;
            case ScenarioMode::STIFF_ENVIRONMENT:
                tau_external = robot.getJacobian(state.q).transpose() * sim.wallContactForce(xy);
                break;
            case ScenarioMode::EXTERNAL_DISTURBANCE:
                tau_external = robot.getJacobian(state.q).transpose() * sim.externalDisturbanceForce(state.time);
                break;
            default:
                break;
        }

        tau += tau_external;

        // Step the simulation
        sim.stepSimulation(robot, tau, state);

        // Update actual end-effector position
        xy = robot.forwardKinematics(state.q);

        q1_deg = state.q(0) * 180 / M_PI;
        q2_deg = state.q(1) * 180 / M_PI;
        q1dot_deg = state.qdot(0) * 180 / M_PI;
        q2dot_deg = state.qdot(1) * 180 / M_PI;
        q1_d_deg = desired_state.q_d(0) * 180 / M_PI;
        q2_d_deg = desired_state.q_d(1) * 180 / M_PI;

        if (state.time + 1e-12 >= next_log_time) {
            logger.writeRow({state.time, q1_deg, q2_deg, q1_d_deg, q2_d_deg, xy(0), xy(1), tau(0), tau(1), xy_d(0), xy_d(1)});
            next_log_time += kSimulationLogInterval;
        }

        if (i % 10000 == 0) {
            std::cout << "t = " << state.time
                      << ", q in degree = [" << q1_deg << ", " << q2_deg << "]"
                      << ", qdot in degree/sec = [" << q1dot_deg << ", " << q2dot_deg << "]"
                      << ", End-effector position = [" << xy(0) << ", " << xy(1) << "]"
                      << std::endl;
        }
    }

    std::cout << "Log written to: " << logger.path() << std::endl;

    return 0;
}
