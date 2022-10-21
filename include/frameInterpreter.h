#ifndef FRAME_INTERPRETER_H
#define FRAME_INTERPRETER_H
#include <cstring>
#include <iostream>
#include <motor.h>
#include <motorInterface.h>
#include <mutex>
#include <stdio.h>
#include <socketcan.h>
#include <string>
#include <thread>
#include <vector>

namespace frameInterpreter {

enum FrameInterpreterError {
    OK,
    SEND_ERROR,
    RECEIVE_ERROR,
    MOTOR_DOES_NOT_EXIST
};

void ReceiveFrame(bool &stop, socketcan::SocketCan &socket_can, std::vector<motor::Motor> &motor_vector, motorInterface::MotorInterface &motor_interface);
void SendFrame(bool &stop, socketcan::SocketCan &socket_can, motorInterface::MotorInterface &motor_interface, std::vector<motor::Motor> &motor_vector);

void ReceiveFromRos(bool &stop, motorInterface::MotorInterface &motor_interface);
void SendToRos(bool &stop, motorInterface::MotorInterface &motor_interface);

void TestRosInterface(motorInterface::MotorInterface &motor_interface);

}

#endif //FRAME_INTERPRETER_H