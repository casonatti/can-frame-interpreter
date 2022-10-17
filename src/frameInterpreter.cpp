#include <frameInterpreter.h>


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

        
        //interpret the frames
        //TODO: preciso do protocolo pra fazer essa parte

        //checksum
        //TODO: implementar

        //select the frames and dispatch
        //if(checkAndSend(message, motor_list) != FrameInterpreterError::OK)
        //    printf("Envio NÃO OK!\n"); //TODO: tratar esse erro!
    }
}

void SendFrame(bool &stop, socketcan::SocketCan &socket_can) {
    socketcan::SocketCanFrame send_frame;
    can_frame message;

    //motorInterface::MotorInterface::readFromInterface();

    motorInterface::MotorInterface::writeToSocketcan(socket_can, message);
}

FrameInterpreterError checkAndSend(can_frame frame, std::list<motor::Motor> motor_list) {
    //for(motor::Motor &it : motor_list) {
    //    if(frame.can_id == it.getID()) {
    //        if(it.readFromSocketcan(frame) >= 0)
    //            return FrameInterpreterError::OK;
    //        else
    //            return FrameInterpreterError::SEND_ERROR;
    //    }
    //}
    //return FrameInterpreterError::MOTOR_DOES_NOT_EXIST;
    return FrameInterpreterError::OK;
}

} //namespace frameInterpreter


int main(int , char *[]) {
    socketcan::SocketCan socket_can;

    std::list<motor::Motor> motor_list;

    motor::Motor m1(0x141, "0");
    motor::Motor m2(0x142, "1");

    motor_list.push_back(m1);
    motor_list.push_back(m2);

    if (socket_can.Open(SOCKETCAN_INTERFACE, RESPONSE_TIMEOUT) != socketcan::SocketCanError::OK) return 0;

    bool stop_all;

    std::thread receive_thread(frameInterpreter::ReceiveFrame, std::ref(stop_all), std::ref(socket_can), std::ref(motor_list));
    std::thread send_thread(frameInterpreter::SendFrame, std::ref(stop_all), std::ref(socket_can));

    receive_thread.join();
    send_thread.join();

    return 0;
}