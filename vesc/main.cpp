#include "mbed.h"
#include "vesc.h"

CAN can1(PA_11, PA_12, 500000);
vesc _vesc1;
BufferedSerial pc(USBTX, USBRX);

int main(){
    _vesc1.vesc_init(&can1,500000);
    _vesc1.set_monitor_id(90);
    while(1){
        printf("position: %d", _vesc1.read_pos(90));
    }
}
