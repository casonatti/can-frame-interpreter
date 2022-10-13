#ifndef MOTOR_H
#define MOTOR_H

#include <chrono>
#include <cstring>
#include <iostream>
#include <socketcan.h>
#include <string>
#include "motor_commands.h"

namespace motor {

enum MotorError {
    OK,
    SEND_ERROR,
    RECEIVE_ERROR
};

enum State {
    NOT_INITIALIZED,
    INITIALIZED
};

struct _joint {
    time_t timestamp;
    std::string name;
    _Float64 position;
    _Float64 velocity;
    _Float64 effort;
};

class Motor {
    private:
        unsigned int id;
        _joint joint;
        std::string name;
        State state;

    public:
        Motor(unsigned int id);
        ~Motor();
        MotorError writeFrame();
        MotorError readFrame(can_frame frame);
        MotorError writeInterface();
        MotorError readInterface(double cmd[]);
        unsigned int getID();
        void setJoint(std::string name, _Float64 position, _Float64 velocity, _Float64 effort);
        _joint getJoint();
        void setName(std::string name);
        std::string getName();
};

} //namespace frameinterpreter

#endif //MOTOR_H