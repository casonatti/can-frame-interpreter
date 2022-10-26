#include <frameInterpreter.h>

#include <unistd.h> //sleep test

namespace frameInterpreter {

std::mutex mtx;

void ReceiveFrame(bool &stop, socketcan::SocketCan &socket_can, std::map<canid_t, motor::Motor> &motor_map, motorInterface::MotorInterface &motor_interface) {
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
            message.can_id |= CAN_RTR_FLAG;
        else
            memcpy(message.data, recv_frame.data, message.len);

        //interpret the frames
        //TODO: preciso do protocolo pra fazer essa parte

        //checksum
        //TODO: implementar

        //update motor value
        motor_interface.readFromSocketcan(message, motor_map);

        //check if all motors have it's velocity calculated
        if(motor_map[0].getMotorDataUpdatedFlag() && motor_map[1].getMotorDataUpdatedFlag())
            motor_interface.setDataToRosFlag(true);
    }
}

void SendFrame(bool &stop, socketcan::SocketCan &socket_can, motorInterface::MotorInterface &motor_interface, std::map<canid_t, motor::Motor> &motor_map) {
    socketcan::SocketCanFrame send_frame_cmd_0, send_frame_cmd_1;
    int send_cnt = -1;
    std::vector<can_frame> frame_vector;

    printf("SendFrame thread initialized.\n");

    while(!stop) {
        if(!motor_interface.getSendFrameFlag()) continue;

        mtx.lock();
        motor_interface.setSendFrameFlag(false);
        
        motor_interface.writeToSocketcan(frame_vector);

        send_frame_cmd_0.can_id = 0x141;

        //TODO: Debug
        printf("Frame 0 = %x#", send_frame_cmd_0.can_id);
        for(int i = 0; i < 8; i++)
            printf("%x ", send_frame_cmd_0.data[i]); 
        printf("\n");

        //TODO: usar o timeout!
        while(send_cnt < 0) {
            send_cnt = socket_can.Write(send_frame_cmd_0, 1);
        }

        //TODO: Debug (simula a resposta do motor 0)
        std::stringstream ss;
        ss << std::hex << std::setfill('0');
        for (int i = 0; i < 8; i++) {
            ss << std::hex << std::setw(2) << static_cast<int>(send_frame_cmd_0.data[i]);
        }
        std::string str_send = "cansend can0 141#" + ss.str();
        std::cout << "cansend can0 " << str_send << std::endl;
        system(str_send.c_str());

        send_frame_cmd_1.can_id = 0x142;

        //TODO: Debug
        printf("Frame 1 = %x#", send_frame_cmd_1.can_id);
        for(int i = 0; i < 8; i++)
            printf("%x ", send_frame_cmd_1.data[i]);
        printf("\n");

        send_cnt = -1;
        while(send_cnt < 0) {
            send_cnt = socket_can.Write(send_frame_cmd_1, 1);
        }

        //TODO: Debug (simula a resposta do motor 1)
        ss.str("");
        ss << std::hex << std::setfill('0');
        for (int i = 0; i < 8; i++) {
            ss << std::hex << std::setw(2) << static_cast<int>(send_frame_cmd_1.data[i]);
        }
        str_send = "cansend can0 142#" + ss.str();
        std::cout << "cansend can0 " << str_send << std::endl;
        system(str_send.c_str());

        send_cnt = -1;

        //TODO: criar um condição específica pra isso
        while(!motor_interface.getDataToRosFlag()); //do nothing

        motor_interface.setMotorDataFlagToFalse(motor_map);

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

void SendToRos(bool &stop, motorInterface::MotorInterface &motor_interface, std::map<canid_t, motor::Motor> &motor_map) {
    while(!stop) {
        if(!motor_interface.getDataToRosFlag()) continue;
        //send to ROS --- como?
        mtx.lock();

        motor_interface.setDataToRosFlag(false);

        //TODO: Debug
        double response[2];
        response[0] = motor_interface.getDoubleResponse(0);
        response[1] = motor_interface.getDoubleResponse(1);
        printf("Cheguei aqui!\n");
        printf("Response[0] = %g\n", response[0]);
        printf("Response[1] = %g\n", response[1]);

        int size = (int) motor_map.size();

        joint::Joint joints[size];

        for(int i = 0; i < size; i++) {
            joints[i] = motor_map[i].getJoint();
            printf("Joint velocity = %g\n", motor_map[i].getJointVelocity());
        }

        mtx.unlock();
    }
}

void TestRosInterface(motorInterface::MotorInterface &motor_interface) {
    double cmd[2];

    sleep(1);

    printf("Enviei o pacote 1...\n");

    cmd[0] = 2;
    cmd[1] = 1;

    motor_interface.readFromInterface(cmd);
        

    sleep(1);

    printf("\nEnviei o pacote 2...\n");

    cmd[0] = 1;
    cmd[1] = 2;

    motor_interface.readFromInterface(cmd);

    sleep(1);

    printf("\nEnviei o pacote 3...\n");

    cmd[0] = 2;
    cmd[1] = 2;

    motor_interface.readFromInterface(cmd);
}

} //namespace frameInterpreter

int main(int , char *[]) {
    socketcan::SocketCan socket_can;
    std::map<canid_t, motor::Motor> motor_map;
    motorInterface::MotorInterface motor_interface;

    motor::Motor m1(0x141, "0");
    motor::Motor m2(0x142, "1");

    motor_map.insert(std::pair<canid_t, motor::Motor>(0x141, m1));
    motor_map.insert(std::pair<canid_t, motor::Motor>(0x142, m2));

    if (socket_can.Open(SOCKETCAN_INTERFACE, RESPONSE_TIMEOUT) != socketcan::SocketCanError::OK) return 0;

    bool stop_all;

    if(motor_interface.initialize(motor_map, socket_can) != motorInterface::MotorError::OK)
        return -1;

    printf("Motors initialized.\n");

    std::thread receive_thread(frameInterpreter::ReceiveFrame, std::ref(stop_all), std::ref(socket_can), std::ref(motor_map), std::ref(motor_interface));
    std::thread send_thread(frameInterpreter::SendFrame, std::ref(stop_all), std::ref(socket_can), std::ref(motor_interface), std::ref(motor_map));

    std::thread receive_from_ros_thread(frameInterpreter::ReceiveFromRos, std::ref(stop_all), std::ref(motor_interface));
    std::thread send_to_ros_thread(frameInterpreter::SendToRos, std::ref(stop_all), std::ref(motor_interface), std::ref(motor_map));

    std::thread ros_interface_test_thread(frameInterpreter::TestRosInterface, std::ref(motor_interface));
    ros_interface_test_thread.join();

    receive_thread.join();
    send_thread.join();
    receive_from_ros_thread.join();
    send_to_ros_thread.join();

    return 0;
}