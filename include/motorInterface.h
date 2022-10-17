#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <cstring>
#include <interface.h>
#include <iostream>
#include <socketcan.h>
#include <string>
#include <motor.h>
#include <motor_commands.h>
#include <vector>

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

class MotorInterface /*: public Interface */ {
    private:
        bool motor_interface_send_flag = false;
        double cmd[2];

    public:
        //constuctor and destructor
        MotorInterface();
        ~MotorInterface();
        
        //convertion
        void doubleToFrameDataConverter(double value, uint8_t frame_data[]);
        double frameDataToDoubleConverter(uint8_t value[]);

        //operation
        int checkWhichMotor(can_frame frame);
        MotorError readFromSocketcan(can_frame frame, std::vector<motor::Motor> &motor);  //receive a new frame from socketcan
        MotorError readFromInterface(double cmd[]);    //receive a new value and operation type from ROS interface
        MotorError writeToSocketcan(can_frame &frame_cmd_m0, can_frame &frame_cmd_m1);   //send a frame to socketcan
        int writeToInterface(double current_state[]);   //send joint current state to ROS interface
        void setSendFrameFlag(bool flag_state);
        bool getSendFrameFlag();
};

} //namespace motorInterface

#endif //MOTOR_INTERFACE_H