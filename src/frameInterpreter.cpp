#include <frameInterpreter.h>


#include <unistd.h> //sleep test

namespace frameInterpreter {

std::mutex mtx;

void ReceiveFrame(bool &stop, socketcan::SocketCan &socket_can, std::vector<motor::Motor> &motor_vector, motorInterface::MotorInterface &motor_interface) {
    socketcan::SocketCanFrame recv_frame;
    can_frame message;

    printf("ReceiveFrame thread initialized.\n");

    while(!stop) {
        int recv_cnt = socket_can.Read(recv_frame, 2);
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

void SendFrame(bool &stop, socketcan::SocketCan &socket_can, motorInterface::MotorInterface &motor_interface, std::vector<motor::Motor> &motor_vector) {
    socketcan::SocketCanFrame send_frame_cmd_0, send_frame_cmd_1;
    int send_cnt = -1;

    printf("SendFrame thread initialized.\n");

    while(!stop) {
        if(!motor_interface.getSendFrameFlag()) continue;

        printf("Setting send frame flag false\n");

        mtx.lock();
        motor_interface.setSendFrameFlag(false);

        //TODO: Debug
        printf("Preparing to write to socketcan...\n");
        
        motor_interface.writeToSocketcan(send_frame_cmd_0, send_frame_cmd_1);

        //TODO: Debug
        printf("Going to send frames:\n");

        send_frame_cmd_0.can_id = 0x141;

        //TODO: Debug
        printf("Frames 1 = %x#", send_frame_cmd_0.can_id);
        for(int i = 0; i < 8; i++)
            printf("%x ", send_frame_cmd_0.data[i]);
        
        printf("\n");

        //TODO: usar o timeout!
        while(send_cnt < 0) {
            send_cnt = socket_can.Write(send_frame_cmd_0, 1);
        }

        system("cansend can0 141#A200000064000000");

        send_frame_cmd_1.can_id = 0x142;

        //TODO: Debug
        printf("Frames 0 = %x#", send_frame_cmd_1.can_id);
        for(int i = 0; i < 8; i++)
            printf("%x ", send_frame_cmd_1.data[i]);

        printf("\n");

        send_cnt = -1;
        while(send_cnt < 0) {
            send_cnt = socket_can.Write(send_frame_cmd_1, 1);
        }

        system("cansend can0 142#A200000064000000");

        send_cnt = -1;

        //TODO: criar um condição específica pra isso
        while(!motor_interface.getDataToRosFlag()); //do nothing

        motor_interface.setMotorDataFlagToFalse(motor_vector);

        //TODO: Debug
        printf("Motor X data updated flag state = %d\n", motor_vector[0].getMotorDataUpdatedFlag());
        printf("Motor XX data updated flag state = %d\n", motor_vector[1].getMotorDataUpdatedFlag());
        mtx.unlock();
    }
}

void ReceiveFromRos(bool &stop, motorInterface::MotorInterface &motor_interface) {
    while(!stop) {
        if(!motor_interface.getNewCommandsFlag()) continue;

        motor_interface.setNewCommandsFlag(false);

        motor_interface.setSendFrameFlag(true);
    }

}

void SendToRos(bool &stop, motorInterface::MotorInterface &motor_interface) {
    while(!stop) {
        if(!motor_interface.getDataToRosFlag()) continue;

        mtx.lock();

        motor_interface.setDataToRosFlag(false);

        double response[2];

        response[0] = motor_interface.getDoubleResponse(0);
        response[1] = motor_interface.getDoubleResponse(1);

        //send to ROS --- como!?!?!?
        printf("Cheguei aqui!\n");
        printf("Response[0] = %g\n", response[0]);
        printf("Response[1] = %g\n", response[1]);

        mtx.unlock();
    }
}

void TestRosInterface(motorInterface::MotorInterface &motor_interface) {
    double cmd[2];

    sleep(3);

    printf("Enviei o pacote 1...\n");

    cmd[0] = 2;
    cmd[1] = 1;

    motor_interface.readFromInterface(cmd);

    sleep(3);

    printf("\nEnviei o pacote 2...\n");

    cmd[0] = 1;
    cmd[1] = 2;

    motor_interface.readFromInterface(cmd);

    sleep(3);

    printf("\nEnviei o pacote 3...\n");

    cmd[0] = 2;
    cmd[1] = 2;

    motor_interface.readFromInterface(cmd);
}

} //namespace frameInterpreter

int main(int , char *[]) {
    socketcan::SocketCan socket_can;
    std::vector<motor::Motor> motor_vector;
    motorInterface::MotorInterface motor_interface;

    motor::Motor m1(0x141, "0");
    motor::Motor m2(0x142, "1");

    motor_vector.push_back(m1);
    motor_vector.push_back(m2);

    if (socket_can.Open(SOCKETCAN_INTERFACE, RESPONSE_TIMEOUT) != socketcan::SocketCanError::OK) return 0;

    bool stop_all;

    if(motor_interface.initialize(motor_vector, socket_can) != motorInterface::MotorError::OK)
        return -1;

    printf("Motors initialized.\n");

    std::thread receive_thread(frameInterpreter::ReceiveFrame, std::ref(stop_all), std::ref(socket_can), std::ref(motor_vector), std::ref(motor_interface));
    std::thread send_thread(frameInterpreter::SendFrame, std::ref(stop_all), std::ref(socket_can), std::ref(motor_interface), std::ref(motor_vector));

    std::thread receive_from_ros_thread(frameInterpreter::ReceiveFromRos, std::ref(stop_all), std::ref(motor_interface));
    std::thread send_to_ros_thread(frameInterpreter::SendToRos, std::ref(stop_all), std::ref(motor_interface));

    std::thread ros_interface_test_thread(frameInterpreter::TestRosInterface, std::ref(motor_interface));
    ros_interface_test_thread.join();

    receive_thread.join();
    send_thread.join();
    receive_from_ros_thread.join();
    send_to_ros_thread.join();

    return 0;
}