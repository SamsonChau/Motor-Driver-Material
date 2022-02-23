#include "mbed.h"
#include "vesc_id.h"

#ifndef VESC_H
#define VESC_H

class vesc
{
public:
    //public functions
    void vesc_init(CAN* _CAN, int baud);                    //init the object  
    void set_monitor_id(int id);                            //set the read status vesc id, currently only support one only

    void set_duty(int id,float duty);                       //set the duty cycle set point for the motor in %
    void set_current(int id, float current);                //set the current set point for the motor in A
    void set_current_brake(int id, float current);          //set the braking current of the motor in A
    void set_rpm(int id, float rpm);                        //set the velocity of the motor in rpm
    void set_pos(int id, float pos);                        //set the position of the motor in deg

    float read_pos(int id);                                 //read the pid position feedback in deg
    float read_rpm(int id);                                 //read the erpm of the motor in rpm
    float read_current(int id);                             //read the current to the motor in A 
    float read_esc_current(int id);                         //read the input bus current in A
    float read_input_voltage(int id);                       //read the input voltage of the bus in V
    float read_fet_temp(int id);                            //read the motor tempeture of the FET in deg C
    void can_read(int id);

private:
    void package_msg(uint8_t* buffer, int32_t number, int32_t *index);
    bool can_send(int id, uint8_t packet[], int32_t len);
    void reset_param();
    
    // internal storage
    int current_id;
    float pid_pos;
    float rpm;
    float current;
    float esc_current;
    float input_voltage;
    float fet_temp;
    
    // Class can msg define
    CAN* can;
    CANMessage Txmsg;
    CANMessage Rxmsg;
};
#endif
