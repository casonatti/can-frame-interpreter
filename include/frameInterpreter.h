#ifndef FRAME_INTERPRETER_H
#define FRAME_INTERPRETER_H
#include <cstring>
#include <iostream>
#include <list>
#include <motor.h>
#include <motorInterface.h>
#include <stdio.h>
#include <socketcan.h>
#include <string>
#include <thread>

namespace frameInterpreter {

enum FrameInterpreterError {
    OK,
    SEND_ERROR,
    RECEIVE_ERROR,
    MOTOR_DOES_NOT_EXIST
};

void ReceiveFrame(bool &stop, socketcan::SocketCan &socket_can, std::list<motor::Motor> &motor_list);
FrameInterpreterError checkAndSend(can_frame frame, std::list<motor::Motor> motor_list);

}

#endif //FRAME_INTERPRETER_H