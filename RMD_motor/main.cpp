#include "mbed.h"
#include "rmd_can.h"

#define TARGET_TX_PIN USBTX
#define TARGET_RX_PIN USBRX
static BufferedSerial serial_port(TARGET_TX_PIN, TARGET_RX_PIN, 115200);

CAN can0(PA_11, PA_12, 1000000);
rmd_can l7015;

int main()
{
    int8_t id = 1;
    uint16_t spd = 60;
    l7015.rmd_can_init(&can0);
    l7015.motor_enable(id);
    
    while (true) {
        //l7015.read_global_angle(id);
        ThisThread::sleep_for(1000);
        //printf("angle: %d\r\n",l7015.pos);
        /*if(l7015.set_zero(id)){
            if(l7015.set_position_speed(id, (uint16_t) 360, spd)){
                printf("cannot set angle: 360\r\n");
                ThisThread::sleep_for(1000);
            };
            if(l7015.set_position_speed(id, (uint16_t) 0, spd)){
                printf("cannot set angle: 0\r\n");
                ThisThread::sleep_for(1000);
            };
        }
        else{
            printf("cannot set zero\r\n");
        }*/
    }
}

