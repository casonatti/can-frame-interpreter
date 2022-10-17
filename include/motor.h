#ifndef MOTOR_H
#define MOTOR_H

#include <iostream>
#include <joint.h>

namespace motor {

enum State {
    NOT_INITIALIZED,
    INITIALIZED
};

class Motor {
    private:
        unsigned int id;
        std::string name;
        joint::Joint joint;
        State state;

    public:
        //constructor and destructor
        Motor(unsigned int id, std::string name);
        ~Motor();

        //getters and setters
        void setName(std::string name);
        void setJoint(std::string name, double position, double velocity, double effort);
        void setVelocity(double velocity);

        unsigned int getID();
        std::string getName();
        std::string getJointName();
        joint::Joint getJoint();
        double getJointEffort();
        double getJointPosition();
        double getJointVelocity();
};

}

#endif //MOTOR_H