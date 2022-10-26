#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <cstring>
#include <interface.h>
#include <iomanip>
#include <iostream>
#include <map>
#include <math.h>
#include <motor.h>
#include <motor_commands.h>
#include <socketcan.h>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <vector>

namespace motorInterface {

enum MotorError {
    OK,
    COULD_NOT_INITIALIZE,
    SEND_TO_SOCKETCAN_ERROR,
    SEND_TO_ROS_INTERFACE_ERROR,
    SEND_TO_MOTOR_ERROR,
    RECEIVE_FROM_ROS_ERROR,
    RECEIVE_FROM_MOTOR_ERROR,
    MOTOR_DOES_NOT_EXIST,
    INVALID_COMMAND
};

enum MotorOperationType {
    VELOCITY,
    POSITION,
    EFFORT
};

class MotorInterface /*: public Interface */ {
    private:
        bool motor_interface_send_flag = false,
                new_commands_flag = false,
                data_to_ros_flag = false;

        double cmd[2], response[2];
        
    public:
        //constuctor and destructor
        MotorInterface();
        ~MotorInterface();

        //operation
        MotorError initialize(std::map<canid_t, motor::Motor> &motor_map, socketcan::SocketCan &socket_can);
        MotorError readFromSocketcan(can_frame frame, std::map<canid_t, motor::Motor> &motor_map);  //receive a new frame from socketcan
        MotorError readFromInterface(double cmd[]);    //receive a new value and operation type from ROS interface
        MotorError writeToSocketcan(std::vector<can_frame> &frame_vector);   //send a frame to socketcan
        MotorError writeToInterface(double current_state[]);   //send joint current state to ROS interface
        
        void setDataToRosFlag(bool flag_state);
        bool getDataToRosFlag();
        void setNewCommandsFlag(bool flag_state);
        bool getNewCommandsFlag();
        void setSendFrameFlag(bool flag_state);
        bool getSendFrameFlag();
        void setDoubleResponse(double value, int n);
        double getDoubleResponse(int n);
        void setMotorDataFlagToFalse(std::map<canid_t, motor::Motor> &motor_map);
};

} //namespace motorInterface

#endif //MOTOR_INTERFACE_H