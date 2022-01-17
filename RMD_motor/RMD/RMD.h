#include "mbed.h"
#ifndef RMD_H
#define RMD_H

#define Header_ID           0x140
#define READ_PID_ID         0x30
#define WRITE_PID_RAM_ID    0x31
#define WRITE_PID_ROM_ID    0x32
#define READ_ACC_ID         0x33
#define WRITE_ACC_ID        0x34
#define READ_ENCODER_ID     0x90
#define WRITE_OFFSET_ID     0x91
#define SET_ZERO_ID         0x19
#define READ_GLOBAL_POS_ID  0x92
#define READ_LOCAL_POS_ID   0x94
#define READ_STATUS_1_ID    0x9A
#define CLEAR_FLAG_ID       0x9B
#define READ_STATUS_2_ID    0x9C
#define READ_STATUS_3_ID    0x9D
#define MOTOR_DISABLE_ID    0x80
#define MOTOR_STOP_ID       0x81
#define MOTOR_ENABLE_ID     0x88
#define TORQ_LOOP_CMD_ID    0xA1
#define VEL_LOOP_CMD_ID     0xA2
#define POS_LOOP_1_CMD_ID   0xA3
#define POS_LOOP_2_CMD_ID   0xA4
#define POS_LOOP_3_CMD_ID   0xA5
#define POS_LOOP_4_CMD_ID   0xA6
#define DEBUG_RMD_LIB_MSG   true 

struct pid{
   int8_t kp;
   int8_t ki;
};

class rmd_can{
    public:
        CAN* can1;
        void rmd_can_init(CAN *_CAN);
        void read_pid(int8_t id);
        void read_acc_pid_data(int8_t id);
        void read_encoder(int8_t id);
        void read_global_angle(int8_t id);
        void read_angle(int8_t id);
    
        void motor_disable(int8_t id);
        void motor_enable(int8_t id);
        void clear_error(int8_t id);

        void status_update1(int8_t id);
        void status_update2(int8_t id);
        void status_update3(int8_t id);
        
        int set_zero(int8_t id);
        int set_torq(int8_t id,int16_t torq);
        int set_velocity(int8_t id,int32_t velocity);
        int set_position(int8_t id,int32_t pos);
        int set_position_speed(int8_t id,int32_t pos,uint16_t profile_speed);
        int set_single_turn_angle(int8_t id,uint16_t angle, uint8_t dir);
        int set_single_turn_angle(int8_t id,uint16_t angle, uint8_t dir, uint16_t profile_speed);
        int set_pid_RAM(int8_t id,int8_t p_kp, int8_t p_ki, int8_t v_kp, int8_t v_ki, int8_t i_kp, int8_t i_ki);
        int set_pid_ROM(int8_t id,int8_t p_kp, int8_t p_ki, int8_t v_kp, int8_t v_ki, int8_t i_kp, int8_t i_ki);
        int set_acc_RAM(int8_t id,int32_t Accel);
        int set_encoder_offset(int8_t id,int16_t pos_offset);

        int8_t temp;
        uint16_t voltage;
        uint8_t error;
        uint16_t encoder;
        uint16_t encoder_raw;
        uint16_t encoder_offset;
        int32_t Accel;// 1 dp/s
        int16_t pos_offset;
        int16_t torq;
        int32_t velocity;
        int64_t pos;
        uint16_t profile_speed;
        uint16_t angle;
        uint8_t dir;
        pid p;
        pid i;
        pid v;

    private:
        //can operation function 
        CANMessage Rx_msg;
        CANMessage Tx_msg;
        int16_t id;
        int8_t cmd[8];
        int8_t data_buff[8];
        int8_t can_send(int16_t id, int8_t data[]);
        int8_t can_read(int16_t id, int8_t cmd);
};
#endif
