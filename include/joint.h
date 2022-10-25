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
        void setTimestamp();
        void setName(std::string name);
        void setEffort(double effort);
        void setPosition(double position);
        void setVelocity(double velocity);

        std::string getName();
        double getPosition();
        double getVelocity();
        double getEffort();
        Joint *getJoint();
};

} //namespace joint

#endif //JOINt_H