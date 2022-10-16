#include <motor.h>

namespace motor {
    Motor::Motor(unsigned int id, std::string name) {
        this->id = id;
        this->name = name;
        this->state = motor::NOT_INITIALIZED;
    }

    Motor::~Motor() {}

    unsigned int Motor::getID() {
        return this->id;
    }

    void Motor::setName(std::string name) {
        this->name = name;
    }

    std::string Motor::getName() {
        return this->name;
    }
}