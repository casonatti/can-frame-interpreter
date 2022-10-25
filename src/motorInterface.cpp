#include <motorInterface.h>

namespace motorInterface {

    MotorInterface::MotorInterface() {}

    MotorInterface::~MotorInterface() {};

    MotorError MotorInterface::initialize(std::vector<motor::Motor> &motor_vector, socketcan::SocketCan &socket_can) {
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
        
        for(int i = 0; i < (int) motor_vector.size(); i++) {
            frame.can_id = motor_vector[i].getID();

            //TODO: melhorar isso... tem que esperar a resposta do motor!
            if(socket_can.Write(frame, 1) != socketcan::SocketCanError::OK) {
                printf("Error on initializing motor. [ID 0x%x", frame.can_id);
                return MotorError::COULD_NOT_INITIALIZE;
            }

            motor_vector[i].setJoint(std::to_string(i), 0, 0, 0);

            motor_vector[i].setState(motor::State::INITIALIZED);
        }

        return MotorError::OK;
    }

    int MotorInterface::checkWhichMotor(can_frame frame) {
        int n = -1;

        if(frame.can_id == 0x141)
            n = 0;
        
        if(frame.can_id == 0x142)
            n = 1;

        return n;
    }

    MotorError MotorInterface::readFromSocketcan(can_frame frame, std::vector<motor::Motor> &motor_vector) {
        //receiving frames from socketcan 
        int motor_n = -1;
        double result[4] = {0};

        motor_n = checkWhichMotor(frame);

        frameDataToDoubleConverter(frame.data, result);

        this->setDoubleResponse(result[2], motor_n);

        //result[] is calculated in frameDataToDoubleConverter() method
        motor_vector[motor_n].setTimestamp();
        motor_vector[motor_n].setEffort(result[1]);
        motor_vector[motor_n].setVelocity(result[2]);
        motor_vector[motor_n].setPosition(result[3]);
        motor_vector[motor_n].setMotorDataUpdatedFlag(true);
        
        return MotorError::OK;
    }

    MotorError MotorInterface::readFromInterface(double cmd[]) {
        //receive commands from ROS interface
        this->cmd[0] = cmd[0];
        this->cmd[1] = cmd[1];

        return MotorError::OK;
    }

    MotorError MotorInterface::writeToSocketcan(can_frame &frame_cmd_m0, can_frame &frame_cmd_m1) {
        uint8_t frame_data_m0[8] = {0};
        uint8_t frame_data_m1[8] = {0};
        double cmd[2];

        cmd[0] = this->cmd[0];
        cmd[1] = this->cmd[1];

        doubleToFrameDataConverter(cmd[0], frame_data_m0);
        doubleToFrameDataConverter(cmd[1], frame_data_m1);

        memcpy(&frame_cmd_m0.data[4], &frame_data_m0[4], 4);
        memcpy(&frame_cmd_m1.data[4], &frame_data_m1[4], 4);

        //velocity command
        frame_cmd_m0.data[0] = SPEED_CLOSED_LOOP_CONTROL;
        frame_cmd_m1.data[0] = SPEED_CLOSED_LOOP_CONTROL;

        for(int i = 3; i > 0; i--) {
            frame_cmd_m0.data[i] = 0x00;
            frame_cmd_m1.data[i] = 0x00;
        }

        frame_cmd_m0.len = 8;
        frame_cmd_m1.len = 8;

        return MotorError::OK;
    }

    MotorError MotorInterface::writeToInterface(double current_state[]) {
        //send motor current status to the ROS interface
        //como fazer??

        return MotorError::OK;
    }

    void MotorInterface::doubleToFrameDataConverter(double value, uint8_t frame_data[]) {
        char data[32] = {0};
        int result_int = -1;

        result_int = (int) (value / DPS_PER_LSB);
        
        //convert int to char[] in hex
        snprintf(data, sizeof(data), "%X", result_int);
        
        std::stringstream sstream;
        sstream << std::hex << (data);
        std::string str_result = sstream.str();

        //check parity
        //needed for endianess conversion further        
        if(str_result.length() % 2 == 1)
            str_result = "0" + str_result;
        
        //convert string to uint8_t
        std::vector<uint8_t> bytes;
        for(int i = 0; i < (int) str_result.length(); i += 2) {
            std::string teste = str_result.substr(i, 2);
            uint8_t byte = (uint8_t) strtol(teste.c_str(), nullptr, 16);
            bytes.push_back(byte);
        }
    
        unsigned int size = bytes.size();
        
        //transforming big endian to little endian (motor RMDX10 V2 protocol format)
        for(int i = 0; i < (int) size; i++) {
            frame_data[4+i] = bytes[size-1-i];
        }
    }
    
    MotorError MotorInterface::frameDataToDoubleConverter(uint8_t frame_data[], double result[]) {
        uint8_t temperature_data, torque_data[2], speed_data[2], encoder_data[2];

        if(frame_data[0] == SPEED_CLOSED_LOOP_CONTROL) {
            temperature_data = frame_data[1];
            torque_data[1] = frame_data[2];
            torque_data[0] = frame_data[3];
            speed_data[1] = frame_data[4];
            speed_data[0] = frame_data[5];
            encoder_data[1] = frame_data[6];
            encoder_data[0] = frame_data[7];

            result[0] = (double)temperature_data;
            result[1] = ((double)(torque_data[0] << 8)) + ((double)(torque_data[1]));
            result[2] = ((double)(speed_data[0] << 8)) + ((double)(speed_data[1]));
            result[3] = ((double)(encoder_data[0] << 8)) + ((double)(encoder_data[1]));

            return MotorError::OK;
        }

        if(frame_data[0] == MOTOR_STOP) {
            return MotorError::OK;
        }

        if(frame_data[0] == SYSTEM_RESET_COMMAND) {
            return MotorError::OK;
        }

        return MotorError::INVALID_COMMAND;
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

    void MotorInterface::setMotorDataFlagToFalse(std::vector<motor::Motor> &motor_vector) {
        for(int i = 0; i < (int) motor_vector.size(); i++)
            motor_vector[i].setMotorDataUpdatedFlag(false);
    }
}