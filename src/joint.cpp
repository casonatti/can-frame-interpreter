#include <joint.h>

namespace joint {
    void Joint::setJoint(std::string name, double position, double velocity, double effort) {
        std::chrono::system_clock::time_point timestamp = std::chrono::system_clock::now();
        this->timestamp = std::chrono::system_clock::to_time_t(timestamp);
        this->name = name;
        this->position = position;
        this->velocity = velocity;
        this->effort = effort;
    }

    void Joint::setName(std::string name) {
        this->name = name;
    }

    void Joint::setEffort(double effort) {
        this->effort = effort;
    }

    void Joint::setPosition(double position) {
        this->position = position;
    }

    void Joint::setVelocity(double velocity) {
        this->velocity = velocity;
    }

    std::string Joint::getName() {
        return this->name;
    }

    double Joint::getEffort() {
        return this->effort;
    }

    double Joint::getPosition() {
        return this->position;
    }

    double Joint::getVelocity() {
        return this->velocity;
    }

    //TODO: testar depois
    Joint *Joint::getJoint() {
            return this;
    }
}