#include "mbed.h"
#include "vesc.h"

/* 
config the vesc in vesc tools before testing
change the setting according your need
the read status function not tested yet
*/

//set the id and can bus baud rate of the VESC
int can_baud = 500000;
int can_id = 90;

CAN can1(PA_11, PA_12, can_baud);   //define the can bus object & vesc object
vesc _vesc1;

int main(){
    //initslize the vesc 
    _vesc1.vesc_init(&can1,can_baud);
    _vesc1.set_monitor_id(can_id);          //This enable the can bus monitor particular id, may improve later
    while(1){
        //reading value
        printf("position: %d \r\n", _vesc1.read_pos(can_id));
        printf("rpm: %d \r\n", _vesc1.read_rpm(can_id));
        printf("current: %d \r\n", _vesc1.read_current(can_id));
        printf("ESC current: %d \r\n", _vesc1.read_esc_current(can_id));
        printf("input_voltage: %d \r\n", _vesc1.read_input_voltage(can_id));
        printf("FET temp: %d \r\n", _vesc1.read_fet_temp(can_id));
    }
}
