#include "odrive_can.h"
#include "mbed.h"
#include <cstdint>

void odrive_can::odrive_init(CAN *_CAN){
    can1 = _CAN;
    can1->frequency(1000000);
    node_id = 0x00;
}

void odrive_can::set_node_id(int8_t new_id){
    node_id = new_id;
    int8_t data_buffer[8] = {new_id,0,0,0,0,0,0,0};
    can_send_data((int16_t)set_node_id_cmd,data_buffer,4);
}
void odrive_can::set_controller_mode(int32_t control_mode, int32_t input_mode){
    int8_t databuffer[8];
    databuffer[0] = (int8_t)control_mode & 0xFF;
    databuffer[1] = (int8_t)control_mode >> 8 & 0xFF;
    databuffer[2] = (int8_t)control_mode >> 16 & 0xFF;
    databuffer[3] = (int8_t)control_mode >> 24 & 0xFF;
    databuffer[4] = (int8_t)input_mode & 0xFF;
    databuffer[5] = (int8_t)input_mode >> 8 & 0xFF;
    databuffer[6] = (int8_t)input_mode >> 16 & 0xFF;
    databuffer[7] = (int8_t)input_mode >> 24 & 0xFF;
    can_send_data((int16_t)set_controller_mode_cmd,databuffer,8);
}
void odrive_can::reboot(){
    can_send_cmd((int16_t) reboot_odrive_cmd);
}
void odrive_can::scan_network(){
    
}
bool odrive_can::checkstate(){

}
void odrive_can::clear_error_flags(){
    can_send_cmd((int16_t)clear_error_flags_cmd);
}
bool odrive_can::estop(){
    if(can_send_cmd((int16_t)odrive_estop_cmd)){
        return true;
    }
    else{
        return false;
    }
}
void odrive_can::set_pos(float pos, int16_t v_ff, int16_t t_ff){
    int32_t position = (int32_t)pos;

    int8_t databuffer[8];
    databuffer[0] = (int8_t)(pos & 0xFF);
    databuffer[1] = (int8_t)(pos >> 8 & 0xFF);
    databuffer[2] = (int8_t)(pos >> 16 & 0xFF);
    databuffer[3] = (int8_t)(pos >> 24 & 0xFF);
    databuffer[4] = (int8_t)v_ff & 0xFF;
    databuffer[5] = (int8_t)v_ff >> 8 & 0xFF;
    databuffer[6] = (int8_t)t_ff >> 16 & 0xFF;
    databuffer[7] = (int8_t)t_ff >> 24 & 0xFF;
    can_send_data((int16_t)set_controller_mode_cmd,databuffer,8);

}
void odrive_can::set_vel(){

}
void odrive_can::set_toque(){

}
void odrive_can::set_limit(){

}

void odrive_can::set_traj_vel_limit(){

}
void odrive_can::set_traj_acc_limit(){

}
void odrive_can::set_traj_intertia(){

}

void odrive_can::set_anticogging(){

}
void odrive_can::set_axis_startup_config(){
    return;
}

void odrive_can::set_axis_request_state(int32_t state){
    int8_t databuffer[8];
    databuffer[0] = (int8_t)state & 0xFF;
    databuffer[1] = (int8_t)state >> 8 & 0xFF;
    databuffer[2] = (int8_t)state >> 16 & 0xFF;
    databuffer[3] = (int8_t)state >> 24 & 0xFF;
    can_send_data((int16_t)set_axis_request_state_cmd,databuffer,4);
}

void odrive_can::set_linear_count(int32_t cnt){
    int8_t databuffer[8];
    databuffer[0] = (int8_t)cnt & 0xFF;
    databuffer[1] = (int8_t)cnt >> 8 & 0xFF;
    databuffer[2] = (int8_t)cnt >> 16 & 0xFF;
    databuffer[3] = (int8_t)cnt >> 24 & 0xFF;
    can_send_data((int16_t)set_linear_count_cmd,databuffer,4);
}
void odrive_can::set_position_gain(){

}
void odrive_can::set_vel_gain(){

}

int32_t odrive_can::get_encoder_count(int8_t mode){
    if(can_send_cmd((int16_t)get_encoder_count_cmd)){
        while(can_read() != (int16_t)get_encoder_count_cmd);
        if(mode == 1){
            return encoder_shadow_cnt;
        }
        else if (mode == 0){
            return encoder_cnt;
        }
        else{
            return 0;
        }
    }
    else{
        return 0; 
    }
}
int32_t odrive_can::get_vbus_voltage(){
    if(can_send_cmd((int16_t)get_vbus_voltage_cmd)){
        while(can_read() != (int16_t)get_vbus_voltage_cmd);
        return vbus;
    }
    else{
        return 0; 
    }
}
int32_t odrive_can::get_sensorless_estimate(int8_t mode){
    if(can_send_cmd((int16_t)get_sensorless_estimate_cmd)){
        while(can_read() != (int16_t)get_sensorless_estimate_cmd);
        if(mode == 1){
            return sensorless_pos_est;
        }
        else if (mode == 0){
            return sensorless_vel_est;
        }
        else{
            return 0;
        }
    }
    else{
        return 0; 
    }
}
int32_t odrive_can::get_encoder_esitimates(int8_t mode){
    if(can_send_cmd((int16_t)get_encoder_esitimates_cmd)){
        while(can_read() != (int16_t)get_encoder_esitimates_cmd);
        if(mode == 1){
            return encoder_pos_est;
        }
        else if (mode == 0){
            return encoder_vel_est;
        }
        else{
            return 0;
        }
    }
    else{
        return 0; 
    }
}
int32_t odrive_can::get_IQ(){
    if(can_send_cmd((int16_t)get_IQ_cmd)){
        while(can_read() != (int16_t)get_IQ_cmd);
        return Iq_measure;
    }
    else{
        return 0; 
    }
}
uint64_t odrive_can::get_motor_error(){
    if(can_send_cmd((int16_t)get_motor_error_cmd)){
        while(can_read() != (int16_t)get_motor_error_cmd);
        return motor_error;
    }
    else{
        return 0; 
    }
}
uint32_t odrive_can::get_encoder_error(){
    if(can_send_cmd((int16_t)get_encoder_error_cmd)){
        while(can_read() != (int16_t)get_encoder_error_cmd);
        return encoder_error;
    }
    else{
        return 0; 
    }
}
uint32_t odrive_can::get_sensorless_error(){
    if(can_send_cmd((int16_t)get_sensorless_error_cmd)){
        while(can_read() != (int16_t)get_sensorless_error_cmd);
        return sensorless_error;
    }
    else{
        return 0; 
    }
}

bool odrive_can::can_send_data(int16_t cmd, int8_t data[8], int8_t framesize){
    CANMessage TxMessage;
    TxMessage.id = (node_id <<5 | cmd);  
    TxMessage.format = CANStandard; 
    TxMessage.type = CANData;      
    TxMessage.len = framesize;   
    for(int i = 0; i < framesize; i++){
        TxMessage.data[i] = data[i];
    }         
    if(can1->write(TxMessage)){
        return true;
    }
    else{
        //printf("CAN SEND CMD ERROR!!")
        return false;
    }
}
bool odrive_can::can_send_cmd(int16_t cmd){
    CANMessage TxMessage;
    TxMessage.id = (node_id <<5 | cmd);  
    TxMessage.format = CANStandard; 
    TxMessage.type = CANData;      
    TxMessage.len = 0;           
    if(can1->write(TxMessage)){
        return true;
    }
    else{
        //printf("CAN SEND CMD ERROR!!")
        return false;
    }
}
int odrive_can::can_read(){
    CANMessage RxMessage;
    can1->read(RxMessage);
    int8_t cmd = (RxMessage.id ^ (node_id << 5))+ 1 ;
    switch (cmd){
        case  heartbeat_msg:{
            axis_error = (uint32_t)(RxMessage.data[3] << 24 | RxMessage.data[2] << 16 | RxMessage.data[1] << 8 | RxMessage.data[0]);
            axis_current_state = (uint8_t)(RxMessage.data[4]);
            axis_controller_state = (uint8_t)(RxMessage.data[7]);
            return heartbeat_msg;
            break;
        }
        case get_motor_error_cmd:{
            motor_error = (uint64_t)((RxMessage.data[7] << 24 | RxMessage.data[6] << 16 | RxMessage.data[5] << 8 | RxMessage.data[4]) | (RxMessage.data[3] << 24 | RxMessage.data[2] << 16 | RxMessage.data[1] << 8 | RxMessage.data[0]);
            return get_motor_error_cmd;
            break;
        }
        case get_IQ_cmd:{
            Iq_set_pt = (int32_t)(RxMessage.data[3] << 24 | RxMessage.data[2] << 16 | RxMessage.data[1] << 8 | RxMessage.data[0]); 
            Iq_measure = (int32_t)(RxMessage.data[7] << 24 | RxMessage.data[6] << 16 | RxMessage.data[5] << 8 | RxMessage.data[4]); 
            return get_IQ_cmd;
            break;
        }
        case get_sensorless_error_cmd:{
            sensorless_error = (uint32_t)(RxMessage.data[3] << 24 | RxMessage.data[2] << 16 | RxMessage.data[1] << 8 | RxMessage.data[0]); 
            return get_sensorless_error_cmd;
            break;
        }
        case get_sensorless_estimate_cmd:{
            sensorless_pos_est = (int32_t)(RxMessage.data[3] << 24 | RxMessage.data[2] << 16 | RxMessage.data[1] << 8 | RxMessage.data[0]);  
            sensorless_vel_est = (int32_t)(RxMessage.data[7] << 24 | RxMessage.data[6] << 16 | RxMessage.data[5] << 8 | RxMessage.data[4]);
            return get_sensorless_estimate_cmd;
            break;
        }
        case get_encoder_error_cmd:{
            encoder_error = (uint32_t)(RxMessage.data[3] << 24 | RxMessage.data[2] << 16 | RxMessage.data[1] << 8 | RxMessage.data[0]); 
            return get_encoder_error_cmd;
            break;
        }
        case get_vbus_voltage_cmd:{
            vbus = (int32_t)(RxMessage.data[3] << 24 | RxMessage.data[2] << 16 | RxMessage.data[1] << 8 | RxMessage.data[0]);  
            return get_vbus_voltage_cmd;
            break;
        }
        case get_encoder_esitimates_cmd:{
            encoder_pos_est = (int32_t)(RxMessage.data[3] << 24 | RxMessage.data[2] << 16 | RxMessage.data[1] << 8 | RxMessage.data[0]); 
            encoder_vel_est = (int32_t)(RxMessage.data[7] << 24 | RxMessage.data[6] << 16 | RxMessage.data[5] << 8 | RxMessage.data[4]); 
            return get_encoder_esitimates_cmd;
            break;
        }
        case get_encoder_count_cmd:{
            encoder_shadow_cnt = (int32_t)(RxMessage.data[3] << 24 | RxMessage.data[2] << 16 | RxMessage.data[1] << 8 | RxMessage.data[0]);  
            encoder_cnt = (int32_t)(RxMessage.data[7] << 24 | RxMessage.data[6] << 16 | RxMessage.data[5] << 8 | RxMessage.data[4]); 
            return get_encoder_count_cmd;
            break;
        }
        default:{
            //printf("Read nothing from node: %d", node_id);
            return 0;
            break;
        }
    };
    return 0; 
}
