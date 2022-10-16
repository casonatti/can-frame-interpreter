#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <cstring>
#include <interface.h>
#include <iostream>
#include <socketcan.h>
#include <string>
#include <motor_commands.h>

namespace motorInterface {

enum MotorError {
    OK,
    SEND_TO_SOCKETCAN_ERROR,
    SEND_TO_ROS_INTERFACE_ERROR,
    SEND_TO_MOTOR_ERROR,
    RECEIVE_FROM_ROS_ERROR,
    RECEIVE_FROM_MOTOR_ERROR,
    MOTOR_DOES_NOT_EXIST
};

enum MotorOperationType {
    VELOCITY,
    POSITION,
    EFFORT
};

class MotorInterface : public Interface {
    private:

    public:
        //constuctor and destructor
        MotorInterface();
        ~MotorInterface();
        
        //convertion
        uint64_t doubleToUint8Converter(double value);
        double uint8ToDoubleConverter(uint8_t value[]);

        //operation
        int checkWhichMotor(can_frame frame);
        double readFromSocketcan(can_frame frame);  //receive a new frame from socketcan
        static int readFromInterface(double cmd[], int type);    //receive a new value and operation type from ROS interface
        static MotorError writeToSocketcan(socketcan::SocketCan &socketcan, can_frame frame);      //send a frame to socketcan
        int writeToInterface(unsigned int motor_id, double current_state, int type);   //send joint current state to ROS interface
};

} //namespace motorInterface

#endif //MOTOR_INTERFACE_H