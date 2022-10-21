#ifndef MOTOR_COMMANDS_H
#define MOTOR_COMMANDS_H

#define READ_POSITION_LOOP_KP_PARAMETER 0x30
#define READ_POSITION_LOOP_KI_PARAMETER 0x31
#define READ_VELOCITY_LOOP_KP_PARAMETER 0x32
#define READ_VELOCITY_LOOP_KI_PARAMETER 0x33
#define READ_TORQUE_LOOP_KP PARAMETER   0x34
#define READ_TORQUE_LOOP_KI_PARAMETER   0x35
#define WRITE_POSITION_LOOP_KP_TO_RAM   0x36
#define WRITE_POSITION_LOOP_KI_TO_RAM   0x37
#define WRITE_VELOCITY_LOOP_KP_TO_RAM   0x38
#define WRITE_VELOCITY_LOOP_KI_TO_RAM   0x39
#define WRITE_TORQUE_LOOP_KP_TO_RAM     0x3A
#define WRITE_TORQUE_LOOP_KI_TO_RAM     0x3B
#define WRITE_POSITION_LOOP_KP_TO_ROM   0x3C
#define WRITE_POSITION_LOOP_KI_TO_ROM   0x3D
#define WRITE_VELOCITY_LOOP_KP_TO_ROM   0x3E
#define WRITE_VELOCITY_LOOP_KI_TO_ROM   0x3F
#define WRITE_TORQUE_LOOP_KP_TO_ROM     0x40
#define wRITE_TORQUE_LOOP_KI_TO_ROM     0x41
#define READ_ACCELERATION               0x42
#define WRITE_ACCELERATION_DATA_TO_RAM  0x43
#define READ_MULTITURN_ENCODER_POSITION 0x60
#define READ_MULTITURN_ENCODER_ORIGINAL_POSITION 0x61
#define READ_MULTITURN_ENCODER_OFFSET   0x62
#define WRITE_MULTITURN_ENCODER_VALUE_TO_ROM_AS_MOTOR_ZERO 0x63
#define WRITE_MULTITURN_ENCODER_CURRENT_POSITION_TO_ROM_AS_MOTOR_ZERO 0x64
#define READ_MULTITURN_TURNS_ANGLE      0x92
#define READ_MOTOR_STATUS_1_AND_ERROR_FLAG 0x9A
#define READ_MOTOR_STATUS_2             0x9C
#define READ_MOTOR_STATUS_3             0x9D
#define MOTOR_OFF                       0x80
#define MOTOR_STOP                      0x81
#define MOTOR_ENABLE                    0x88
#define TORQUE_CLOSED_LOOP_CONTROL      0xA1
#define SPEED_CLOSED_LOOP_CONTROL    0xA2
#define MULTI_POSITION_CLOSED_LOOP_CONTROL 0xA3
#define INCREMENTAL_POSITION_CLOSED_LOOP_CONTROL 0xA8
#define READ_MOTOR_CURRENT_WORKING_MODE 0x70
#define READ_MOTOR_CURRENT_POWER        0x71
#define READ_BACKUP_POWER_VOLTAGE       0x72
#define TF_COMMAND                      0x73
#define SYSTEM_RESET_COMMAND            0x76
#define BRAKE_UNLOCK_COMMAND            0x77 //motor turn freely
#define BRAKE_LOCK                      0x78 //motor shaft locked
#define SETUP_AND_READ_CAN_ID           0x79
#define READ_SYSTEM_RUNNING_TIME        0xB1
#define READ_FIRMWARE_VERSION           0xB2
#define SETUP_COMMUNICATION_INTERRUPTION_PROTECTION_TIME 0xB3

#endif //MOTOR_COMMANDS_H