#include <motorInterface.h>

namespace motorInterface {

    MotorInterface::MotorInterface() {}

    MotorInterface::~MotorInterface() {};

    int MotorInterface::checkWhichMotor(can_frame frame) {
        int n = -1;

        if(frame.can_id == 0x141)
            n = 0;
        
        if(frame.can_id == 0x142)
            n = 1;

        return n;
    }

    double MotorInterface::readFromSocketcan(can_frame frame) {
        //receiving frames from socketcan 
        double result;
        
        result = uint8ToDoubleConverter(frame.data);
        
        return result;
    }

    int MotorInterface::readFromInterface(double cmd[], int type) {
        //receive commands from the ROS interface
        can_frame result;
        int x = -1;
        double result;

        return MotorError::OK;
    }

    MotorError writeToSocketcan(socketcan::SocketCan &socketcan, can_frame frame) {
        //send frames to socketcan

        return MotorError::OK;
    }

    int MotorInterface::writeToInterface(unsigned int motor_id, double current_state, int type) {
        //send motor current status to the ROS interface
        //como fazer??

        return MotorError::OK;
    }
}