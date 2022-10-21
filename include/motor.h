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
        bool motor_data_updated = false;

    public:
        //constructor and destructor
        Motor(unsigned int id, std::string name);
        ~Motor();

        //getters and setters
        void setName(std::string name);
        void setJoint(std::string name, double position, double velocity, double effort);
        void setState(State state);
        void setVelocity(double velocity);

        unsigned int getID();
        std::string getName();
        std::string getJointName();
        joint::Joint getJoint();
        double getJointEffort();
        double getJointPosition();
        double getJointVelocity();
        State getstate();

        void setMotorDataUpdatedFlag(bool flag_state);
        bool getMotorDataUpdatedFlag();
};

}

#endif //MOTOR_H