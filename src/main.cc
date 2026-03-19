#include "robot_model.hpp"
#include <iostream>
#include <cmath>

int main() {
    // Create a robot model with link lengths of 1.0 and 1.0
    RobotModel robot(1.0, 1.0, 1.0, 1.0);

    // Set joint angles (in radians)
    double q1 = M_PI/2;
    double q2 = 0;
    robot.setJointAngles(q1, q2);

    // Compute forward kinematics
    double x, y;
    robot.forwardKinematics(x, y);

    double x_com1, y_com1;
    double x_com2, y_com2;
    Eigen::Vector2d com1 = robot.getCOM1Position();
    Eigen::Vector2d com2 = robot.getCOM2Position();

    // Print the end-effector position
    std::cout << "End-effector position: (" << x << ", " << y << ")" << std::endl;
    std::cout << "Center of mass 1 position: (" << com1.x() << ", " << com1.y() << ")" << std::endl;
    std::cout << "Center of mass 2 position: (" << com2.x() << ", " << com2.y() << ")" << std::endl;

    return 0;
}
