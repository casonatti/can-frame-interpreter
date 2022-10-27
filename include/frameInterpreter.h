#ifndef FRAME_INTERPRETER_H
#define FRAME_INTERPRETER_H

#include <chrono>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <map>
#include <motor.h>
#include <motorManager.h>
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

void ReceiveFrame(bool &stop, socketcan::SocketCan &socket_can, std::map<unsigned int, motor::Motor> &motor_map, motorManager::MotorManager &motor_interface);
void SendFrame(bool &stop, socketcan::SocketCan &socket_can, motorManager::MotorManager &motor_interface, std::map<unsigned int, motor::Motor> &motor_map);

void SendToRos(bool &stop, motorManager::MotorManager &motor_interface, std::map<unsigned int, motor::Motor> &motor_map);

void TestRosInterface(motorManager::MotorManager &motor_interface, std::map<canid_t, motor::Motor> &motor_map);

}

#endif //FRAME_INTERPRETER_H