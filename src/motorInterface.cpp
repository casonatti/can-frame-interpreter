#include <motorInterface.h>

namespace motorInterface {

    MotorInterface::MotorInterface() {}

    MotorInterface::~MotorInterface() {};

    MotorError MotorInterface::initialize(std::map<unsigned int, motor::Motor> &motor_map, socketcan::SocketCan &socket_can) {
        can_frame frame;

        frame.data[0] = 0x88;
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;

        frame.len = 8;
        
        for(auto &it : motor_map) {
            frame.can_id = it.second.getID();
        
            //TODO: melhorar isso... tem que esperar a resposta do motor!
            if(socket_can.Write(frame, 1) != socketcan::SocketCanError::OK) {
                printf("Error on initializing motor. [ID 0x%x]", frame.can_id);
                return MotorError::COULD_NOT_INITIALIZE;
            }

            it.second.setJoint(it.second.getName(), 0, 0, 0);

            it.second.setState(motor::State::INITIALIZED);
        }

        return MotorError::OK;
    }

    MotorError MotorInterface::readFromInterface(double cmd[], std::map<canid_t, motor::Motor> &motor_map) {
        //receive commands from ROS interface
        if(this->getNewCommandsFlag())
            return MotorError::UNABLE_TO_GET_NEW_COMMANDS;

        int i = 0;
        for(auto &it : motor_map) {
            it.second.setCommand(cmd[i]);
            i++;
        }

        this->setNewCommandsFlag(true);
        this->setSendFrameFlag(true);

        return MotorError::OK;
    }

    MotorError MotorInterface::writeToInterface(std::map<canid_t, motor::Motor> motor_map) {
        //send motor current status to the ROS interface
        
        int size = (int) motor_map.size();

        joint::Joint joints[size];

        int i=0;
        for(auto &it : motor_map) {
            joints[i] = it.second.getJoint();
            i++;
            printf("Joint velocity = %g\n", it.second.getJointVelocity());
        }

        return MotorError::OK;
    }

    MotorError MotorInterface::produceDouble(can_frame frame, std::map<unsigned int, motor::Motor> &motor_map) {
        //receiving frames from socketcan 
        if(motor_map.find(frame.can_id)->second.frameDataToDoubleConverter(frame.data)) {
            motor_map.find(frame.can_id)->second.setMotorDataUpdatedFlag(true);
            return MotorError::OK;
        }
        
        return MotorError::CONVERTION_ERROR;
    }

    MotorError MotorInterface::produceFrame(std::vector<can_frame> &frame_vector, std::map<canid_t, motor::Motor> &motor_map) {
        uint8_t frame_data[8] = {0};
        double cmd[motor_map.size()];

        //fill cmd[] with each motor command
        int i = 0;
        for(auto &it : motor_map) {
            frame_vector[i].can_id = it.second.getID();
            cmd[i] = it.second.getCommand();
            i++;
        }

        int size = (int) frame_vector.size();

        for(int i = 0; i < size; i++) {
            motor::Motor::doubleToFrameDataConverter(cmd[i], frame_data);

            memcpy(&frame_vector[i].data[4], &frame_data[4], 4);

            //velocity command
            frame_vector[i].data[0] = SPEED_CLOSED_LOOP_CONTROL;

            for(int j = 3; j > 0; j--) {
                frame_vector[i].data[j] = 0x00;
            }

            frame_vector[i].len = 8;
        }

        return MotorError::OK;
    }

    void MotorInterface::setDataToRosFlag(bool flag_state) {
        this->data_to_ros_flag = flag_state;
    }

    bool MotorInterface::getDataToRosFlag() {
        return this->data_to_ros_flag;
    }

    void MotorInterface::setSendFrameFlag(bool flag_state) {
        this->motor_interface_send_flag = flag_state;
    }

    bool MotorInterface::getSendFrameFlag() {
        return this->motor_interface_send_flag;
    }

    void MotorInterface::setNewCommandsFlag(bool flag_state) {
        this->new_commands_flag = flag_state;
    }

    bool MotorInterface::getNewCommandsFlag() {
        return this->new_commands_flag;
    }

    void MotorInterface::setDoubleResponse(double value, int n){
        this->response[n] = value;
    }

    double MotorInterface::getDoubleResponse(int n) {
        return this->response[n];
    }

    void MotorInterface::setMotorDataFlagToFalse(std::map<unsigned int, motor::Motor> &motor_map) {
        for(auto &it : motor_map)
            it.second.setMotorDataUpdatedFlag(false);
    }
}