// Declare robot model class

class RobotModel {
public:
    RobotModel(double link1_length, double link2_length, double link1_mass, double link2_mass);
    void setJointAngles(double q1, double q2);
    void forwardKinematics(double& x, double& y) const;

private:
    const double l1_; // Length of the first link
    const double l2_; // Length of the second link
    // Assume mass is uniformly distributed along the link
    const double m1_; // Mass of the first link
    const double m2_; // Mass of the second link
    double q1_; // Joint angle of the first link
    double q2_; // Joint angle of the second link
};