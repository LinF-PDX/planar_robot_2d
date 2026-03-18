#include "robot_model.hpp"
#include <iostream>
#include <cmath>

int main() {
    // Create a robot model with link lengths of 1.0 and 1.0
    RobotModel robot(1.0, 1.0, 1.0, 1.0);

    // Set joint angles (in radians)
    double q1 = M_PI/4;
    double q2 = M_PI/2;
    robot.setJointAngles(q1, q2);

    // Compute forward kinematics
    double x, y;
    robot.forwardKinematics(x, y);

    // Print the end-effector position
    std::cout << "End-effector position: (" << x << ", " << y << ")" << std::endl;

    return 0;
}
