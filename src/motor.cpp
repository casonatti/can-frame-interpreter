#include <motor.h>

namespace motor {

    Motor::Motor(unsigned int id) {
        this->id = id;
        this->state = motor::NOT_INITIALIZED;
    };

    Motor::~Motor() {};

    MotorError Motor::writeFrame() {
        //sending frames to socket

        return MotorError::OK;
    }

    MotorError Motor::readFrame(can_frame frame) {
        //receiving frames from socket
        
        return MotorError::OK;
    }

    MotorError Motor::writeInterface() {
        
    }

    MotorError Motor::readInterface(double cmd[]) {

    }

    unsigned int Motor::getID() {
        return this->id;
    }

    void Motor::setJoint(std::string name, _Float64 position, _Float64 velocity, _Float64 effort) {
        std::chrono::system_clock::time_point timestamp = std::chrono::system_clock::now();
        this->joint.timestamp = std::chrono::system_clock::to_time_t(timestamp);
        this->joint.name = name;
        this->joint.position = position;
        this->joint.velocity = velocity;
        this->joint.effort = effort;
    }

    _joint Motor::getJoint() {
        return this->joint;
    }

    void Motor::setName(std::string name) {
        this->name = name;
    }

    std::string Motor::getName() {
        return this->name;
    }
}