#include "mbed.h"
#include "vesc_id.h"

#ifndef VESC_H
#define VESC_H

class vesc
{
public:
    void vesc_init(CAN* _CAN, int baud);                        //init the object  
    void set_monitor_id(int id);                            //set the read status vesc id, currently only support one only

    void set_duty(int id,float duty);                       //set the duty cycle set point for the motor
    void set_current(int id, float current);                //set the current set point for the motor
    void set_current_brake(int id, float current);          //set the braking current of the motor
    void set_rpm(int id, float rpm);                        //set the velocity of the motor
    void set_pos(int id, float pos);                        //set the position of the motor

    float read_pos(int id);                               //read the pid position feedback 
    float read_rpm(int id);                               //read the erpm of the motor
    float read_current(int id);                           //read the current to the motor
    float read_esc_current(int id);                       //read the input bus current
    float read_input_voltage(int id);                     //read the input voltage of the bus
    float read_fet_temp(int id);                          //read the motor tempeture of the FET
    void can_read(int id);

private:
    void package_msg(uint8_t* buffer, int32_t number, int32_t *index);
    bool can_send(int id, uint8_t packet[], int32_t len);
    void reset_param();

    int current_id;
    float pid_pos;
    float rpm;
    float current;
    float esc_current;
    float input_voltage;
    float fet_temp;

    CAN* can;
    CANMessage Txmsg;
    CANMessage Rxmsg;
};
#endif