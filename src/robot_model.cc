#include "robot_model.hpp"

// Define RobotModel class methods
RobotModel::RobotModel(double link1_length, double link2_length, double link1_mass, double link2_mass)
    :   l1_(link1_length), 
        l2_(link2_length), 
        m1_(link1_mass), 
        m2_(link2_mass), 
        lc1_(link1_length / 2.0), 
        lc2_(link2_length / 2.0),
        I1_((1.0 / 12.0) * link1_mass * link1_length * link1_length),
        I2_((1.0 / 12.0) * link2_mass * link2_length * link2_length),
        g_(9.81) {
}

Eigen::Vector2d RobotModel::forwardKinematics(const Eigen::Vector2d& q) const {
    // Compute the position of the end-effector using forward kinematics
    // Given q, find x and y
    double x, y;
    x = l1_ * std::cos(q(0)) + l2_ * std::cos(q(0) + q(1));
    y = l1_ * std::sin(q(0)) + l2_ * std::sin(q(0) + q(1));
    return Eigen::Vector2d(x, y);
}

Eigen::Vector2d RobotModel::inverseKinematics(double x, double y) const {
    // Compute the joint angles using inverse kinematics
    // Given x and y, find q

    // Check if the target position is reachable
    double q1_temp, q1_temp2, q2_temp, q2_temp2;
    double distance = std::sqrt(x * x + y * y);
    if (distance > l1_ + l2_ || distance < std::abs(l1_ - l2_) || (x < 0)) {
        throw std::runtime_error("Target position is out of reach");
    }
    double c2 = (x * x + y * y - l1_ * l1_ - l2_ * l2_) / (2 * l1_ * l2_);

    // Clamp c2 to the valid range [-1, 1] to avoid numerical issues with acos
    c2 = std::max(-1.0, std::min(1.0, c2));

    // Two possible solutions for q2
    q2_temp = std::acos(c2);
    q2_temp2 = -std::acos(c2);

    // Two possible solutions for q1
    q1_temp = std::atan2(y, x) - std::atan2(l2_ * std::sin(q2_temp), l1_ + l2_ * std::cos(q2_temp));
    q1_temp2 = std::atan2(y, x) - std::atan2(l2_ * std::sin(q2_temp2), l1_ + l2_ * std::cos(q2_temp2));

    // Reject invalid solutions
    if (-M_PI / 2 <= q1_temp && q1_temp <= M_PI / 2) {
        return Eigen::Vector2d(q1_temp, q2_temp);
    } else if (-M_PI / 2 <= q1_temp2 && q1_temp2 <= M_PI / 2) {
        return Eigen::Vector2d(q1_temp2, q2_temp2);
    } else {
        throw std::runtime_error("No valid joint angles found for the target position");
    }
}

Eigen::Vector2d RobotModel::getCOM1Position(const Eigen::Vector2d& q) const {
    // Compute the position of the center of mass of the first link
    double x = lc1_ * std::cos(q(0));
    double y = lc1_ * std::sin(q(0));
    return Eigen::Vector2d(x, y);
}

Eigen::Vector2d RobotModel::getCOM2Position(const Eigen::Vector2d& q) const {
    // Compute the position of the center of mass of the second link
    double x = l1_ * std::cos(q(0)) + lc2_ * std::cos(q(0) + q(1));
    double y = l1_ * std::sin(q(0)) + lc2_ * std::sin(q(0) + q(1));
    return Eigen::Vector2d(x, y);
}

Eigen::Matrix2d RobotModel::getMassMatrix(const Eigen::Vector2d& q) const {
    // Compute the mass matrix of the robot
    double c2 = std::cos(q(1));
    Eigen::Matrix2d M;
    M(0, 0) = I1_ + I2_ + m1_ * lc1_ * lc1_ + m2_ * (l1_ * l1_ + lc2_ * lc2_ + 2 * l1_ * lc2_ * c2);
    M(0, 1) = I2_ + m2_ * (lc2_ * lc2_ + l1_ * lc2_ * c2);
    M(1, 0) = M(0, 1);
    M(1, 1) = I2_ + m2_ * lc2_ * lc2_;
    return M;
}

Eigen::Vector2d RobotModel::getGravityVector(const Eigen::Vector2d& q) const {
    // Compute the gravity vector of the robot
    double c1 = std::cos(q(0));
    double c2 = std::cos(q(0) + q(1));
    Eigen::Vector2d G;
    G(0) = (m1_ * lc1_ + m2_ * l1_) * g_ * c1 + m2_ * lc2_ * g_ * c2;
    G(1) = m2_ * lc2_ * g_ * c2;
    return G;
}

Eigen::Vector2d RobotModel::getCoriolisVector(const Eigen::Vector2d& q, const Eigen::Vector2d& qdot) const {
    // Compute the Coriolis vector of the robot
    double s2 = std::sin(q(1));
    double c2 = std::cos(q(1));
    double h = m2_ * l1_ * lc2_ * s2;
    Eigen::Vector2d C;
    C(0) = -h * qdot(1) * (2 * qdot(0) + qdot(1));
    C(1) = h * qdot(0) * qdot(0);
    return C;
}

Eigen::Vector2d RobotModel::forwardDynamics(const Eigen::Vector2d& q, 
                                            const Eigen::Vector2d& qdot, 
                                            const Eigen::Vector2d& tau) const {
    // Compute the joint accelerations using forward dynamics
    Eigen::Vector2d qddot;
    Eigen::Matrix2d M = getMassMatrix(q);
    Eigen::Vector2d C = getCoriolisVector(q, qdot);
    Eigen::Vector2d G = getGravityVector(q);
    qddot = M.inverse() * (tau - C - G);
    return qddot;
}