#include <iostream>
#include "logger.hpp"
#include "simulator.hpp"
#include "controller.hpp"

#define SIMULATION_TOTAL_TIME_SECOND 10.0
#define SIMULATION_TIME_STEP 0.0001
#define SIMULATION_LOG_INTERVAL 0.01
#define ROBOT_LINK1_LENGTH 1.0
#define ROBOT_LINK2_LENGTH 1.0
#define ROBOT_LINK1_MASS 1.0
#define ROBOT_LINK2_MASS 1.0

int main() {
    RobotModel robot(ROBOT_LINK1_LENGTH, ROBOT_LINK2_LENGTH, ROBOT_LINK1_MASS, ROBOT_LINK2_MASS);
    Simulator sim(SIMULATION_TIME_STEP);
    const double log_interval = SIMULATION_LOG_INTERVAL;

    RobotState state;
    state.q << -M_PI/6, 0.0;
    state.qdot << 0.0, 0.0;

    // Convert radians to degrees for logging
    double q1_deg = state.q(0) * 180 / M_PI;
    double q2_deg = state.q(1) * 180 / M_PI;
    double q1dot_deg = state.qdot(0) * 180 / M_PI;
    double q2dot_deg = state.qdot(1) * 180 / M_PI;

    Eigen::Vector2d tau;
    tau << 0.0, 0.0;

    Eigen::Vector2d xy = robot.forwardKinematics(state.q);

    // Initialize logger
    SignalLogger logger("data", {"time", "q1", "q2", "q1dot", "q2dot", "x", "y", "tau1", "tau2"});

    // Write first log entry
    double next_log_time = 0.0;
    logger.writeRow({state.time, q1_deg, q2_deg, q1dot_deg, q2dot_deg, xy(0), xy(1), tau(0), tau(1)});
    next_log_time += log_interval;

    for (int i = 0; i < SIMULATION_TOTAL_TIME_SECOND / SIMULATION_TIME_STEP; ++i) {
        sim.stepSimulation(robot, tau, state);
        xy = robot.forwardKinematics(state.q);

        q1_deg = state.q(0) * 180 / M_PI;
        q2_deg = state.q(1) * 180 / M_PI;
        q1dot_deg = state.qdot(0) * 180 / M_PI;
        q2dot_deg = state.qdot(1) * 180 / M_PI;

        if (state.time + 1e-12 >= next_log_time) {
            logger.writeRow({state.time, q1_deg, q2_deg, q1dot_deg, q2dot_deg,
                             xy(0), xy(1), tau(0), tau(1)});
            next_log_time += log_interval;
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
