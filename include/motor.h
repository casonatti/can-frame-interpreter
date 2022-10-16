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
        Motor::Motor(unsigned int id, std::string name);
        Motor::~Motor();

        //getters and setters
        unsigned int getID();
        void setName(std::string name);
        std::string getName();
};

}

#endif //MOTOR_H