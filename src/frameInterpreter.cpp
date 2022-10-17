#include <frameInterpreter.h>


#include <unistd.h> //sleep test

namespace frameInterpreter {

void ReceiveFrame(bool &stop, socketcan::SocketCan &socket_can, std::vector<motor::Motor> &motor_vector) {
    motorInterface::MotorInterface motor_interface;
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

        //update motor value
        motor_interface.readFromSocketcan(message, motor_vector);
    }
}

void SendFrame(bool &stop, socketcan::SocketCan &socket_can) {
    motorInterface::MotorInterface motor_interface;
    socketcan::SocketCanFrame send_frame_cmd_0, send_frame_cmd_1;
    can_frame message_m0, message_m1;
    int send_cnt = -1;

    while(!stop) {
        while(!motor_interface.getSendFrameFlag()); //do nothing
        
        motor_interface.writeToSocketcan(message_m0, message_m1);

        send_frame_cmd_0.can_id = 0x141;
        send_frame_cmd_0.len = message_m0.len;

        while(send_cnt < 0) {
            send_cnt = socket_can.Write(send_frame_cmd_0, 1);
        }

        send_frame_cmd_1.can_id = 0x142;
        send_frame_cmd_1.len = message_m1.len;

        send_cnt = -1;
        while(send_cnt < 0) {
            send_cnt = socket_can.Write(send_frame_cmd_1, 1); 
        }
    }
}

void ReceiveFromRos() {

}

void SendToRos() {

}

} //namespace frameInterpreter


int main(int , char *[]) {
    socketcan::SocketCan socket_can;

    std::vector<motor::Motor> motor_vector;

    motor::Motor m1(0x141, "0");
    motor::Motor m2(0x142, "1");

    motor_vector.push_back(m1);
    motor_vector.push_back(m2);

    if (socket_can.Open(SOCKETCAN_INTERFACE, RESPONSE_TIMEOUT) != socketcan::SocketCanError::OK) return 0;

    bool stop_all;

    std::thread receive_thread(frameInterpreter::ReceiveFrame, std::ref(stop_all), std::ref(socket_can), std::ref(motor_vector));
    std::thread send_thread(frameInterpreter::SendFrame, std::ref(stop_all), std::ref(socket_can));

    std::thread receive_from_ros_thread(frameInterpreter::ReceiveFromRos);
    std::thread send_to_ros_thread(frameInterpreter::SendToRos);

    receive_thread.join();
    send_thread.join();
    receive_from_ros_thread.join();
    send_to_ros_thread.join();

    return 0;
}