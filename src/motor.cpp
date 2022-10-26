#include <motor.h>

namespace motor {
    Motor::Motor(unsigned int id, std::string name) {
        this->id = id;
        this->name = name;
        this->state = motor::NOT_INITIALIZED;
    }

    Motor::~Motor() {}

    unsigned int Motor::getID() {
        return this->id;
    }

    void Motor::setName(std::string name) {
        this->name = name;
    }

    void Motor::setState(State state) {
        this->state = state;
    }

    void Motor::setEffort(double effort) {
        this->joint.setEffort(effort);
    }

    void Motor::setPosition(double position) {
        this->joint.setPosition(position);
    }

    void Motor::setVelocity(double velocity) {
        this->joint.setVelocity(velocity);
    }

    void Motor::setJoint(std::string name, double position, double velocity, double effort) {
        this->joint.setName(name);
        this->joint.setPosition(position);
        this->joint.setVelocity(velocity);
        this->joint.setEffort(effort);
    }

    void Motor::setCommand(double cmd) {
        this->command = cmd;
    }

    std::string Motor::getName() {
        return this->name;
    }

    joint::Joint Motor::getJoint() {
        joint::Joint result;
        result.setName(this->joint.getName());
        result.setEffort(this->joint.getEffort());
        result.setPosition(this->joint.getPosition());
        result.setVelocity(this->joint.getVelocity());

        return result;
    }

    double Motor::getJointEffort() {
        return this->joint.getEffort();
    }

    double Motor::getJointPosition() {
        return this->joint.getPosition();
    }

    double Motor::getJointVelocity() {
        return this->joint.getVelocity();
    }

    void Motor::setMotorDataUpdatedFlag(bool flag_state) {
        this->motor_data_updated = flag_state;
    }
    
    bool Motor::getMotorDataUpdatedFlag() {
        return this->motor_data_updated;
    }

    State Motor::getState() {
        return this->state;
    }

    double Motor::getCommand() {
        return this->command;
    }

    int Motor::frameDataToDoubleConverter(uint8_t frame_data[]) {
        uint8_t temperature_data, torque_data[2], speed_data[2], encoder_data[2];
        double result[4];

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

            this->setEffort(result[1]);
            this->setVelocity(result[2]);
            this->setPosition(result[3]);
            this->setMotorDataUpdatedFlag(true);

            return 1;
        }

        if(frame_data[0] == MOTOR_STOP) {
            return 1;
        }

        if(frame_data[0] == SYSTEM_RESET_COMMAND) {
            return 1;
        }

        return 0;
    }

    void Motor::doubleToFrameDataConverter(double value, uint8_t frame_data[]) {
        char data[32] = {0};
        int result_int = -1;

        result_int = (int) (value);/* / DPS_PER_LSB); */
        
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
}