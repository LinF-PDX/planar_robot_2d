#include "robot_model.hpp"
#include <cmath>

// Define RobotModel class methods
RobotModel::RobotModel(double link1_length, double link2_length, double link1_mass, double link2_mass)
    :   l1_(link1_length), 
        l2_(link2_length), 
        m1_(link1_mass), 
        m2_(link2_mass), 
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