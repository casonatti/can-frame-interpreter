#include <motorInterface.h>

namespace motorInterface {

    MotorInterface::MotorInterface() {}

    MotorInterface::~MotorInterface() {};

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
        int motor_n;
        double result;

        motor_n = checkWhichMotor(frame);

        result = frameDataToDoubleConverter(frame.data);

        motor_vector[motor_n].setVelocity(result);
        
        return MotorError::OK;
    }

    MotorError MotorInterface::readFromInterface(double cmd[]) {
        //receive commands from the ROS interface
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

        return MotorError::OK;
    }

    int MotorInterface::writeToInterface(double current_state[]) {
        //send motor current status to the ROS interface
        //como fazer??

        return MotorError::OK;
    }

    void MotorInterface::setSendFrameFlag(bool flag_state) {
        this->motor_interface_send_flag = flag_state;
    }

    bool MotorInterface::getSendFrameFlag() {
        return this->motor_interface_send_flag;
    }

    void doubleToFrameDataConverter(double value, uint8_t frame_data[]) {
        
    }
    
    double frameDataToDoubleConverter(uint8_t value[]) {
        double result;

        return result;
    }
}