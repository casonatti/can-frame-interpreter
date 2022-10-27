#include <frameInterpreter.h>

#include <unistd.h> //sleep test

using namespace std::chrono_literals;

namespace frameInterpreter {

void ReceiveFrame(bool &stop, socketcan::SocketCan &socket_can, std::map<unsigned int, motor::Motor> &motor_map, motorManager::MotorManager &motor_interface) {
    socketcan::SocketCanFrame recv_frame;
    can_frame message;

    printf("ReceiveFrame thread initialized.\n");
    
    while(!stop) {
        int recv_cnt = socket_can.Read(recv_frame, 2);
        if(recv_cnt < 0) return;    //check if CAN interface is not up
        if(!recv_cnt) continue;
       
        //receive frame
        message.can_id = recv_frame.can_id & CAN_EFF_MASK;
        message.can_dlc = recv_frame.can_dlc;

        //check rtr flag
        if(recv_frame.can_id & CAN_RTR_FLAG)
            message.can_id |= CAN_RTR_FLAG;
        else
            memcpy(message.data, recv_frame.data, message.can_dlc);

        //checksum
        //TODO: preciso do protocolo pra implementar

        //update motor value
        motor_interface.produceDouble(message, motor_map);

        //check if all motors have it's velocity calculated
        int motor_data_ok = 0;
        int size = (int) motor_map.size();
        for(auto &it : motor_map)
            if(it.second.getMotorDataUpdatedFlag())
                motor_data_ok++;
        
        if(motor_data_ok == size) {
            motor_interface.setDataToRosFlag(true);
        }
    }
}

void SendFrame(bool &stop, socketcan::SocketCan &socket_can, motorManager::MotorManager &motor_interface, std::map<unsigned int, motor::Motor> &motor_map) {
    int send_cnt = -1;
    std::vector<can_frame> frame_vector;

    //there will be 1 frame for each instance of motors
    frame_vector.resize(motor_map.size());

    int size = (int) frame_vector.size();

    printf("SendFrame thread initialized.\n");

    while(!stop) {
        if(!motor_interface.getSendFrameFlag()) continue;

        motor_interface.setSendFrameFlag(false);
        
        //convert commands to frames
        motor_interface.produceFrame(frame_vector, motor_map);

        send_cnt = -1;

        for(int i = 0; i < size; i++) {
            while(send_cnt < 0)
                send_cnt = socket_can.Write(frame_vector[i], 1);

            //TODO: Debug (simula a resposta do motor)
            /* std::stringstream ss, id_hex;
            ss << std::hex << std::setfill('0');
            id_hex << std::hex << frame_vector[i].can_id;
            for (int i = 0; i < 8; i++) {
                ss << std::hex << std::setw(2) << static_cast<int>(frame_vector[0].data[i]);
            }
            std::string str_send = "cansend can0 " + id_hex.str() + "#" + ss.str();
            system(str_send.c_str()); */

            send_cnt = -1;
        }
    }
}

void SendToRos(bool &stop, motorManager::MotorManager &motor_interface, std::map<unsigned int, motor::Motor> &motor_map) {
    while(!stop) {
        if(!motor_interface.getDataToRosFlag()) continue;
        //send to ROS

        motor_interface.setDataToRosFlag(false);

        motor_interface.writeToInterface(motor_map);    //send joints to ros interface

        motor_interface.setMotorDataFlagToFalse(motor_map); //set motor data updated flags of all motors to false

        motor_interface.setNewCommandsFlag(false);  //enable the program to receive new commands
    }
}

/* void TestRosInterface(motorManager::MotorManager &motor_interface, std::map<canid_t, motor::Motor> &motor_map) {
    double cmd[2];
    auto d1 = 10ms;
    sleep(1);

    printf("Enviei o pacote 1...\n");

    cmd[0] = 2;
    cmd[1] = 1;

    while(motor_interface.readFromInterface(cmd, motor_map) == motorManager::MotorError::UNABLE_TO_GET_NEW_COMMANDS) {
        printf("WARNING: calculating last commands!\n");
        std::this_thread::sleep_for(d1);
    }

    std::this_thread::sleep_for(d1);

    printf("\nEnviei o pacote 2...\n");

    cmd[0] = 1;
    cmd[1] = 2;

    while(motor_interface.readFromInterface(cmd, motor_map) == motorManager::MotorError::UNABLE_TO_GET_NEW_COMMANDS) {
        printf("WARNING: calculating last commands!\n");
        std::this_thread::sleep_for(d1);
    }

    std::this_thread::sleep_for(d1);

    printf("\nEnviei o pacote 3...\n");

    cmd[0] = 2;
    cmd[1] = 2;

    while(motor_interface.readFromInterface(cmd, motor_map) == motorManager::MotorError::UNABLE_TO_GET_NEW_COMMANDS) {
        printf("WARNING: calculating last commands!\n");
        std::this_thread::sleep_for(d1);
    }
} */

} //namespace frameInterpreter

int main(int , char *[]) {
    socketcan::SocketCan socket_can;
    std::map<unsigned int, motor::Motor> motor_map;
    motorManager::MotorManager motor_interface;

    motor::Motor m1(0x141, "0");
    motor::Motor m2(0x142, "1");
    

    motor_map.insert(std::pair<unsigned int, motor::Motor>(0x141, m1));
    motor_map.insert(std::pair<unsigned int, motor::Motor>(0x142, m2));
    
    

    if (socket_can.Open(SOCKETCAN_INTERFACE, RESPONSE_TIMEOUT) != socketcan::SocketCanError::OK) return 0;

    bool stop_all;

    if(motor_interface.initialize(motor_map, socket_can) != motorManager::MotorError::OK)
        return -1;

    printf("Motors initialized.\n");

    std::thread receive_thread(frameInterpreter::ReceiveFrame, std::ref(stop_all), std::ref(socket_can), std::ref(motor_map), std::ref(motor_interface));
    std::thread send_thread(frameInterpreter::SendFrame, std::ref(stop_all), std::ref(socket_can), std::ref(motor_interface), std::ref(motor_map));

    std::thread send_to_ros_thread(frameInterpreter::SendToRos, std::ref(stop_all), std::ref(motor_interface), std::ref(motor_map));

    //std::thread ros_interface_test_thread(frameInterpreter::TestRosInterface, std::ref(motor_interface), std::ref(motor_map));
    //ros_interface_test_thread.join();

    receive_thread.join();
    send_thread.join();
    send_to_ros_thread.join();

    return 0;
}