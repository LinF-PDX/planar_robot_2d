#include "robot_model.hpp"
#include <cmath>

// Define RobotModel class methods
RobotModel::RobotModel(double link1_length, double link2_length, double link1_mass, double link2_mass)
    :   l1_(link1_length), 
        l2_(link2_length), 
        m1_(link1_mass), 
        m2_(link2_mass), 
        lc1_(link1_length / 2.0), 
        lc2_(link2_length / 2.0),
        I1_((1.0 / 12.0) * link1_mass * pow(link1_length, 2)),
        I2_((1.0 / 12.0) * link2_mass * pow(link2_length, 2)),
        g_(9.81),
        q1_(0.0), 
        q2_(0.0) {
}

void RobotModel::setJointAngles(double q1, double q2) {
    q1_ = q1;
    q2_ = q2;
}

void RobotModel::forwardKinematics(double& x, double& y) const {
    // Compute the position of the end-effector using forward kinematics
    x = l1_ * cos(q1_) + l2_ * cos(q1_ + q2_);
    y = l1_ * sin(q1_) + l2_ * sin(q1_ + q2_);
}

Eigen::Vector2d RobotModel::getCOM1Position() const {
    // Compute the position of the center of mass of the first link
    double x = lc1_ * cos(q1_);
    double y = lc1_ * sin(q1_);
    return Eigen::Vector2d(x, y);
}

Eigen::Vector2d RobotModel::getCOM2Position() const {
    // Compute the position of the center of mass of the second link
    double x = l1_ + lc2_ * cos(q1_ + q2_);
    double y = lc2_ * sin(q1_ + q2_);
    return Eigen::Vector2d(x, y);
}