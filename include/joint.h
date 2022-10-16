#ifndef JOINT_H
#define JOINT_H

#include <iostream>
#include <chrono>

namespace joint {

class Joint {
    private:
        time_t timestamp;
        std::string name;
        double position;
        double velocity;
        double effort;
    public:
        void setJoint(std::string name, double position, double velocity, double effort);
        std::string getName();
        double getPosition();
        double getVelocity();
        double getEffort();
        Joint *getJoint();
};

} //namespace joint

#endif //JOINt_H