#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <cstring>
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
    INVALID_COMMAND,
    UNABLE_TO_GET_NEW_COMMANDS,
    CONVERTION_ERROR
};

enum MotorOperationType {
    VELOCITY,
    POSITION,
    EFFORT
};

class MotorInterface {
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
        MotorError initialize(std::map<unsigned int, motor::Motor> &motor_map, socketcan::SocketCan &socket_can);
        MotorError readFromInterface(double cmd[], std::map<canid_t, motor::Motor> &motor_map);    //receive a new value and operation type from ROS interface
        std::vector<joint::Joint> writeToInterface(std::map<canid_t, motor::Motor> motor_map);   //send joint current state to ROS interface
        void produceDouble(can_frame frame, std::map<unsigned int, motor::Motor> &motor_map);  //receive a new frame from socketcan
        void produceFrame(std::vector<can_frame> &frame_vector, std::map<canid_t, motor::Motor> &motor_map);   //send a frame to socketcan
        
        void setDataToRosFlag(bool flag_state);
        bool getDataToRosFlag();
        void setNewCommandsFlag(bool flag_state);
        bool getNewCommandsFlag();
        void setSendFrameFlag(bool flag_state);
        bool getSendFrameFlag();
        void setDoubleResponse(double value, int n);
        double getDoubleResponse(int n);
        void setMotorDataFlagToFalse(std::map<unsigned int, motor::Motor> &motor_map);
};

} //namespace motorInterface

#endif //MOTOR_INTERFACE_H