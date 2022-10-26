#ifndef MOTOR_H
#define MOTOR_H

#include <iostream>
#include <joint.h>
#include <motor_commands.h>
#include <string>
#include <sstream>
#include <vector>

#define DPS_PER_LSB 0.01

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
        void setEffort(double effort);
        void setPosition(double position);
        void setVelocity(double velocity);
        unsigned int getID();
        std::string getName();
        std::string getJointName();
        joint::Joint getJoint();      
        double getJointEffort();
        double getJointPosition();
        double getJointVelocity();
        State getState();
        void setMotorDataUpdatedFlag(bool flag_state);
        bool getMotorDataUpdatedFlag();

        //operational
        int frameDataToDoubleConverter(uint8_t frame_data[]);
        static void doubleToFrameDataConverter(double value, uint8_t frame_data[]);
};

}

#endif //MOTOR_H