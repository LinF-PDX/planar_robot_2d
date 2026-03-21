#include <iostream>
#include "robot_model.hpp"

int main() {
    // Create a robot model with link lengths of 1.0 and 1.0
    RobotModel robot(1.0, 1.0, 1.0, 1.0);

    // Compute inverse kinematics
    double x, y;
    x = std::sqrt(3);
    y = -1.0;
    //robot.forwardKinematics(x, y, q);
    Eigen::Vector2d q = robot.inverseKinematics(x, y);

    Eigen::Vector2d com1 = robot.getCOM1Position(q);
    Eigen::Vector2d com2 = robot.getCOM2Position(q);
    Eigen::Vector2d gravity_vector = robot.getGravityVector(q);

    // Print the end-effector position
    std::cout << "End-effector position: (" << x << ", " << y << ")" << std::endl;
    std::cout << "Joint angles in degree: (" << q.x() * 180 / M_PI << ", " << q.y() * 180 / M_PI << ")" << std::endl;
    std::cout << "Center of mass 1 position: (" << com1.x() << ", " << com1.y() << ")" << std::endl;
    std::cout << "Center of mass 2 position: (" << com2.x() << ", " << com2.y() << ")" << std::endl;
    std::cout << "Gravity vector: (" << gravity_vector.x() << ", " << gravity_vector.y() << ")" << std::endl;

    return 0;
}
