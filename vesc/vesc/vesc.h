#include "mbed.h"
#include "vesc_id.h"

#ifndef VESC_H
#define VESC_H

class vesc
{
public:
    void vesc_init(CAN* _CAN, int baud);                        //init the object  
    void set_monitor_id(uint8_t id);                            //set the read status vesc id, currently only support one only

    void set_duty(uint8_t id,float duty);                       //set the duty cycle set point for the motor
    void set_current(uint8_t id, float current);                //set the current set point for the motor
    void set_current_brake(uint8_t id, float current);          //set the braking current of the motor
    void set_rpm(uint8_t id, float rpm);                        //set the velocity of the motor
    void set_pos(uint8_t id, float pos);                        //set the position of the motor

    int16_t read_pos(uint8_t id);                               //read the pid position feedback 
    int32_t read_rpm(uint8_t id);                               //read the erpm of the motor
    int16_t read_current(uint8_t id);                           //read the current to the motor
    int16_t read_esc_current(uint8_t id);                       //read the input bus current
    int16_t read_input_voltage(uint8_t id);                     //read the input voltage of the bus
    int16_t read_fet_temp(uint8_t id);                          //read the motor tempeture of the FET
    void can_read();

private:
    void package_msg(uint8_t* buffer, int32_t number, int32_t *index);
    bool can_send(uint32_t id, uint8_t packet[], int32_t len);
    void reset_param();

    uint8_t current_id;
    int16_t pid_pos;
    int32_t rpm;
    int16_t current;
    int16_t esc_current;
    int16_t input_voltage;
    int16_t fet_temp;

    CAN* can;
    CANMessage Txmsg;
    CANMessage Rxmsg;
};
#endif