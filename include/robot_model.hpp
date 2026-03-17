// Declare robot model class

class RobotModel {
public:
    RobotModel(double link1_length, double link2_length);
    void setJointAngles(double q1, double q2);
    void forwardKinematics(double& x, double& y) const;

private:
    double l1_; // Length of the first link
    double l2_; // Length of the second link
    double q1_; // Joint angle of the first link
    double q2_; // Joint angle of the second link
};