#include <iostream>
#include "robot_model.hpp"
#include "simulator.hpp"

int main() {
    // Create a robot model with link lengths of 1.0 and 1.0
    RobotModel robot(1.0, 1.0, 1.0, 1.0);
    Simulator sim(0.001);

    RobotState state;
    state.q << -M_PI/4, M_PI/4; // Initial joint angles
    state.qdot << 0.0, 0.0; // Initial joint velocities
    state.time = 0.0; // Initial time

    Eigen::Vector2d tau;
    tau << 0.0, 0.0;

    for (int i = 0; i < 10000; ++i) {
        sim.stepSimulation(robot, tau, state);

        // Only print every 1000 steps to reduce output
        if (i % 10 == 0) {
            std::cout << "t = " << state.time
                      << ", q in degree = [" << state.q(0) * 180 / M_PI << ", " << state.q(1) * 180 / M_PI << "]"
                      << ", qdot in degree/sec = [" << state.qdot(0) * 180 / M_PI << ", " << state.qdot(1) * 180 / M_PI << "]"
                      << std::endl;
        }
    }

    // Compute inverse kinematics
    // double x, y;
    // x = 2;
    // y = 0;
    // //robot.forwardKinematics(x, y, q);
    // Eigen::Vector2d q = robot.inverseKinematics(x, y);

    // Eigen::Vector2d com1 = robot.getCOM1Position(q);
    // Eigen::Vector2d com2 = robot.getCOM2Position(q);
    // Eigen::Vector2d gravity_vector = robot.getGravityVector(q);

    // // Print the end-effector position
    // std::cout << "End-effector position: (" << x << ", " << y << ")" << std::endl;
    // std::cout << "Joint angles in degree: (" << q.x() * 180 / M_PI << ", " << q.y() * 180 / M_PI << ")" << std::endl;
    // std::cout << "Center of mass 1 position: (" << com1.x() << ", " << com1.y() << ")" << std::endl;
    // std::cout << "Center of mass 2 position: (" << com2.x() << ", " << com2.y() << ")" << std::endl;
    // std::cout << "Gravity vector: (" << gravity_vector.x() << ", " << gravity_vector.y() << ")" << std::endl;

    return 0;
}
