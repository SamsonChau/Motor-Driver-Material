#include "mbed.h"
#include "vesc.h"

void vesc::vesc_init(CAN* _CAN, int baud){
    can = _CAN;
    can -> frequency(baud);
    can -> attach(callback(this, &vesc::can_read), CAN::RxIrq);
}

void vesc::set_monitor_id(uint8_t id){
    if(current_id != id){
        current_id = id;
        reset_param();
    }
}

void vesc::package_msg(uint8_t* buffer, int32_t number, int32_t *index) {
   buffer[(*index)++] = number >> 24;
   buffer[(*index)++] = number >> 16;
   buffer[(*index)++] = number >> 8;
   buffer[(*index)++] = number;
}

bool vesc::can_send(uint32_t id, uint8_t packet[], int32_t len){
   Txmsg.id = id;
   Txmsg.format = CANExtended;
   Txmsg.type = CANData;
   Txmsg.len = len; 

   for(int i = 0; i < len; i++){
       Txmsg.data[i] = packet[i];
   }
   
   if (can->write(Txmsg)){   
      return true;
   }
   else {
      return false;
   }
}

void vesc::reset_param(){
    current_id = 0;
    pid_pos = 0;
    rpm = 0;
    current = 0;
    esc_current = 0;
    input_voltage = 0;
    fet_temp = 0;
}

//Set vesc command
void vesc::set_duty(uint8_t id, float duty) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    package_msg(buffer, (int32_t)(duty * 100000.0), &send_index);
    can_send(id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

void vesc::set_current(uint8_t id, float current) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    package_msg(buffer, (int32_t)(current * 1000.0), &send_index);
    can_send(id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

void vesc::set_current_brake(uint8_t id, float current) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    package_msg(buffer, (int32_t)(current * 1000.0), &send_index);
    can_send(id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

void vesc::set_rpm(uint8_t id, float rpm) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    package_msg(buffer, (int32_t)rpm, &send_index);
    can_send(id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

void vesc::set_pos(uint8_t id, float pos) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    package_msg(buffer, (int32_t)(pos * 1000000.0), &send_index);
    can_send(id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

//read vesc command
void vesc::can_read(){
    can -> read(Rxmsg);
    uint8_t can_id = (uint8_t)Rxmsg.id & 0xFF;
    CAN_PACKET_ID cmd = (CAN_PACKET_ID)((uint8_t)Rxmsg.id >> 8);
    if(can_id == current_id){
        switch (cmd) {
            case CAN_PACKET_STATUS:
				rpm = (((uint32_t) Rxmsg.data[0]) << 24 | ((uint32_t) Rxmsg.data[1]) << 16 | ((uint32_t) Rxmsg.data[2]) << 8 | ((uint32_t) Rxmsg.data[4]));
				esc_current = (((uint16_t) Rxmsg.data[4]) << 8 | ((uint16_t) Rxmsg.data[5]))/ 10.0;
				break;
            
            case CAN_PACKET_STATUS_4:
                fet_temp = (((uint16_t) Rxmsg.data[0]) << 8 | ((uint16_t) Rxmsg.data[1]))/ 10.0;
				current = (((uint16_t) Rxmsg.data[4]) << 8 | ((uint16_t) Rxmsg.data[5]))/ 10.0;
				pid_pos = (((uint16_t) Rxmsg.data[6]) << 8 | ((uint16_t) Rxmsg.data[7]))/ 50.0;
                break;
            
            case CAN_PACKET_STATUS_5:
				input_voltage = (((uint16_t) Rxmsg.data[4]) << 8 | ((uint16_t) Rxmsg.data[5]))/ 10.0;
                break;
            default:
                break;
        }
    }
}

int16_t vesc::read_pos(uint8_t id){
    if(id != current_id){
        current_id = id;
        reset_param();
        //printf("updated monitor esc id");
        return 0;
    }
    else{
        return pid_pos;
    }
}
int32_t vesc::read_rpm(uint8_t id){
    if(id != current_id){
        current_id = id;
        reset_param();
        //printf("updated monitor esc id");
        return 0;
    }
    else{
        return rpm;
    }
}
int16_t vesc::read_current(uint8_t id){
    if(id != current_id){
        current_id = id;
        reset_param();
        //printf("updated monitor esc id");
        return 0;
    }
    else{
        return esc_current;
    }
}
int16_t vesc::read_esc_current(uint8_t id){
    if(id != current_id){
        current_id = id;
        reset_param();
        //printf("updated monitor esc id");
        return 0;
    }
    else{
        return esc_current;
    }
}
int16_t vesc::read_input_voltage(uint8_t id){
    if(id != current_id){
        current_id = id;
        reset_param();
        //printf("updated monitor esc id");
        return 0;
    }
    else{
        return input_voltage;
    }
}
int16_t vesc::read_fet_temp(uint8_t id){
    if(id != current_id){
        current_id = id;
        reset_param();
        //printf("updated monitor esc id");
        return 0;
    }
    else{
        return fet_temp;
    }
}
