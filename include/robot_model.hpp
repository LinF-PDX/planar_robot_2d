// Declare robot model class
#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

class RobotModel {
public:
    RobotModel(double link1_length, double link2_length, double link1_mass, double link2_mass);
    Eigen::Vector2d forwardKinematics(const Eigen::Vector2d& q) const;
    Eigen::Vector2d inverseKinematics(double x, double y) const;

    Eigen::Vector2d getCOM1Position(const Eigen::Vector2d& q) const;
    Eigen::Vector2d getCOM2Position(const Eigen::Vector2d& q) const;

    Eigen::Matrix2d getMassMatrix(const Eigen::Vector2d& q) const;
    Eigen::Vector2d getGravityVector(const Eigen::Vector2d& q) const;
    Eigen::Vector2d getCoriolisVector(const Eigen::Vector2d& q, const Eigen::Vector2d& qdot) const;

    Eigen::Vector2d forwardDynamics(const Eigen::Vector2d& q, 
                                    const Eigen::Vector2d& qdot, 
                                    const Eigen::Vector2d& tau) const;

    double getTotalEnergy(const Eigen::Vector2d& q, const Eigen::Vector2d& qdot) const;

private:
    const double l1_; // Length of the first link
    const double l2_; // Length of the second link
    const double m1_; // Mass of the first link
    const double m2_; // Mass of the second link
    const double lc1_; // Distance from the first joint to the center of mass of the first link
    const double lc2_; // Distance from the second joint to the center of mass of the second link
    const double I1_; // Moment of inertia of the first link about its center of mass
    const double I2_; // Moment of inertia of the second link about its center of mass
    const double g_; // Gravitational acceleration
};