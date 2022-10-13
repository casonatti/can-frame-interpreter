#include <cstring>
#include <iostream>
#include <frameInterpreter.h>
#include <list>
#include <motor.h>
#include <stdio.h>
#include <socketcan.h>
#include <string>
#include <thread>

#include <unistd.h> //sleep test

namespace frameInterpreter {

void ReceiveFrame(bool &stop, socketcan::SocketCan &socket_can, std::list<motor::Motor> &motor_list) {
    socketcan::SocketCanFrame recv_frame;
    can_frame message;

    while(!stop) {
        int recv_cnt = socket_can.Read(recv_frame, 1);
        if(recv_cnt < 0) return;    //check if CAN interface is not up
        if(!recv_cnt) continue;
       
        //receive frame
        message.can_id = recv_frame.can_id & CAN_EFF_MASK;
        message.len = recv_frame.can_dlc;

        //check rtr flag
        if(recv_frame.can_id & CAN_RTR_FLAG)
            printf("RTR = 1 [true]\n"); //TODO: verificar essa parte (como funciona o rtr nesse caso?)
        else
            printf("RTR = 0 [false]\n");
        memcpy(message.data, recv_frame.data, message.len);

        //TODO: Debug (Apagar depois)
        printf("***************************************************\n");
        printf("Message ID: %x\n", message.can_id);
        printf("Message length: %x\n", message.len);
        printf("Message: ");
        for(int i = 0; i < message.len; i++)
            printf("%x ", message.data[i]);
        

        //interpret the frames
        //TODO: preciso do protocolo pra fazer essa parte

        //select the frames and dispatch
        if(checkAndSend(message, motor_list) != FrameInterpreterError::OK)
            printf("\nEnvio NÃO OK!"); //TODO: tratar esse erro!
        
        printf("\n***************************************************\n\n");
    }
}

FrameInterpreterError checkAndSend(can_frame frame, std::list<motor::Motor> motor_list) {
    for(motor::Motor &it : motor_list) {
        if(frame.can_id == it.getID()) {
            if(it.readFrame(frame) == motor::MotorError::OK)
                return FrameInterpreterError::OK;
            else
                return FrameInterpreterError::SEND_ERROR;
        }
    }
    return FrameInterpreterError::MOTOR_DOES_NOT_EXIST;
}

} //namespace frameInterpreter


int main(int , char *[]) {
    socketcan::SocketCan socket_can;

    std::list<motor::Motor> motor_list;

    motor::Motor m1(0x140);
    motor::Motor m2(0x141);

    m1.setName("Motor 1");
    m2.setName("Motor 2");

    motor_list.push_back(m1);
    motor_list.push_back(m2);

    if (socket_can.Open(SOCKETCAN_INTERFACE, RESPONSE_TIMEOUT) != socketcan::SocketCanError::OK) return 0;

    bool stop_all;

    std::thread receive_thread(frameInterpreter::ReceiveFrame, std::ref(stop_all), std::ref(socket_can), std::ref(motor_list));

    receive_thread.join();

    return 0;
}