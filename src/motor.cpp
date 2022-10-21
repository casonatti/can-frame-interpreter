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

    void Motor::setState(State state) {
        this->state = state;
    }

    void Motor::setVelocity(double velocity) {
        this->joint.setVelocity(velocity);
    }

    std::string Motor::getName() {
        return this->name;
    }

    joint::Joint Motor::getJoint() {
        joint::Joint result;
        result.setName(this->joint.getName());
        result.setEffort(this->joint.getEffort());
        result.setPosition(this->joint.getPosition());
        result.setVelocity(this->joint.getVelocity());

        return result;
    }

    void Motor::setMotorDataUpdatedFlag(bool flag_state) {
        this->motor_data_updated = flag_state;
    }
    
    bool Motor::getMotorDataUpdatedFlag() {
        return this->motor_data_updated;
    }
}