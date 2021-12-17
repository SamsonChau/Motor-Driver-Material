#include "mbed.h"
#include "RMD.h"

void rmd_can::rmd_can_init(CAN *_CAN){
    can1 = _CAN;
    can1->frequency(1000000);
    id = ;
    data_buff[8] = {};
    temp = 0;
    voltage = 0;
    error = 0;
    p.kp, p.ki = 0;
    v.kp, v.ki = 0;
    i.kp, i.ki = 0;
    motor_disable();
    motor_set_zero();
}
int rmd_can::can_Send(int16_t id, int8_t data[]){
    int cnt = 0;
    TxMessage.id = Header_ID + id;         
    TxMessage.format = CANStandard; 
    TxMessage.type = CANData;       
    TxMessage.len = 8;             
    TxMessage.data[0] = data[0]; 
    TxMessage.data[1] = data[1];     
    TxMessage.data[2] = data[2];
    TxMessage.data[3] = data[3];      
    TxMessage.data[4] = data[4]; 
    TxMessage.data[5] = data[5];      
    TxMessage.data[6] = data[6]; 
    TxMessage.data[7] = data[7];     
    while(!can1->write(TxMessage)|| counter <= 3){
        printf("sent command fail! retry %d\r\n", cnt);
    }
    if(can1->write(TxMessage)){
        printf("Sent cmd!\r\n");
        return 0;
    }
    else{
        printf("sent command fail!\r\n");
        return -1;
    }
    return 0;
}

int8_t rmd_can::can_read(int16_t id, int8_t cmd){
    int cnt = 0;
    can1->read(RX_msg);
    printf("RX msg receieved!\r\n");
    printf("msg ID: 0x%x\r\n",RX_msg.id);
    if(RX_msg.id != id){
        printf("Device ID error\r\n!");
        return -1;
    }
    else if(RX_msg.data[0] != cmd){
        printf("cmd error\r\n!");
        while(RX_msg.data[0] != cmd || cnt <= 4){
            can1->read(RX_msg);
            printf("RX msg receieved!\r\n");
            printf("msg ID: 0x%x\r\n",RX_msg.id);
            printf("cmd ID: 0x%x\r\n",RX_msg.data[0]);
            printf("msg error retry attempt %d\r\n",cnt);
            cnt++;
        }
        if(RX_msg.data[0] != cmd){
            printf("RX msg ERROR!\r\n");
            printf("msg ID: 0x%x\r\n",RX_msg.id);
            printf("cmd ID: 0x%x\r\n",RX_msg.data[0]);
            prinf("Device noot found!\r\n");
            return -1
        }else{
            for (int i=0; i<8; i++){
                data_buff[i] = Rx_msg.data[i];
            }
            return Rx_msg.data[0];
        }
    }
    return 0;
}

void rmd_can::read_pid(int8_t id){
    int8_t cmd[8] = {};
    cmd[0] = READ_PID_ID; 
    if(can_send(id,cmd)){
        int8_t reply = 0;
        reply = can_read(id,READ_PID_ID);
        if(reply == READ_PID_ID){
            p.kp = data_buff[2];
            p.ki = data_buff[3];
            v.kp = data_buff[4];
            v.ki = data_buff[5];
            i.kp = data_buff[6];
            i.ki = data_buff[7];
            printf("PID param updated!\r\n");
            //return 0;
        }    
        else{
            printf("PID param readed fail!\r\n");
            //return -1; 
        }
    };
    //return 0;
}
void rmd_can::read_acc_pid_data(int8_t id){
    int8_t cmd[8] = {};
    cmd[0] = READ_ACC_ID; 
    if(can_send(id,cmd)){
        int8_t reply = 0;
        reply = can_read(id,READ_ACC_ID);
        if(reply == READ_ACC_ID){
            Accel = (int32_t)(data_buff[4] | data_buff[5] << 8 | data_buff[5] << 16 | data_buff[5] << 24);
            printf("ACCEL param updated!\r\n");
            //return 0;
        }    
        else{
            printf("ACCEL param readed fail!\r\n");
            //return -1; 
        }
    }
    //return 0;
}
void rmd_can::read_encoder(int8_t id){
    int8_t cmd[8] = {};
    cmd[0] = READ_ENCODER_ID; 
    if(can_send(id,cmd)){
        int8_t reply = 0;
        reply = can_read(id,READ_ENCODER_ID);
        if(reply == READ_ENCODER_ID){
            encoder = (uint16_t)(data_buff[2] | data_buff[3] << 8 );
            encoder_raw = (uint16_t)(data_buff[4] | data_buff[5] << 8);
            encoder_raw = (uint16_t)(data_buff[6] | data_buff[7] << 8);
            printf("Encoder value updated!\r\n");
            //return 0;
        }    
        else{
            printf("ACCEL param readed fail!\r\n");
            //return -1; 
        }
    }
    //return 0;
}
void rmd_can::read_global_angle(int8_t id){
    int8_t cmd[8] = {};
    cmd[0] = READ_ENCODER_ID; 
    if(can_send(id,cmd)){
        int8_t reply = 0;
        reply = can_read(id,READ_ENCODER_ID);
        if(reply == READ_ENCODER_ID){   
            pos = (int64_t)(data_buff[1] | data_buff[2] << 8 || data_buff[3] << 16 || data_buff[4] << 24 || data_buff[5] << 32 | data_buff[6] << 40 | data_buff[7] << 48 );
            printf("global position updated!\r\n");
            //return 0;
        }    
        else{
            printf("global position readed fail!\r\n");
            //return -1; 
        }
    }
    //return 0;
}
void rmd_can::read_angle(int8_t id);

void rmd_can::motor_disable(int8_t id);
void rmd_can::motor_enable(int8_t id);
void rmd_can::clear_error(int8_t id);

void rmd_can::status_update1(int8_t id);
void rmd_can::status_update2(int8_t id);
void rmd_can::status_update3(int8_t id);

int rmd_can::set_zero(int8_t id);
int rmd_can::set_torq(int8_t id,int16_t torq);
int rmd_can::set_velocity(int8_t id,int32_t velocity);
int rmd_can::set_position(int8_t id,int32_t pos);
int rmd_can::set_position_speed(int8_t id,int32_t pos,uint16_t profile_speed);
int rmd_can::set_single_turn_angle(int8_t id,uint16_t angle, uint8_t dir);
int rmd_can::set_single_turn_angle(int8_t id,uint16_t angle, uint8_t dir, uint16_t profile_speed);
int rmd_can::set_pid_RAM(int8_t id,int8_t p_kp, int8_t p_ki, int8_t v_kp, int8_t v_ki, int8_t i_kp, int8_t i_ki);
int rmd_can::set_pid_ROM(int8_t id,int8_t p_kp, int8_t p_ki, int8_t v_kp, int8_t v_ki, int8_t i_kp, int8_t i_ki);
int rmd_can::set_acc_RAM(int8_t id,int32_t Accel);
int rmd_can::set_encoder_offset(int8_t id,int16_t pos_offset);

