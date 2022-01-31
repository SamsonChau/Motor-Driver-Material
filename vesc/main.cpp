#include "mbed.h"
#include "vesc.h"
#define TARGET_TX_PIN                                                     USBTX
#define TARGET_RX_PIN                                                     USBRX
static BufferedSerial serial_port(TARGET_TX_PIN, TARGET_RX_PIN, 115200);
/*
config the vesc in vesc tools before testing
change the setting according your need
the read status function not tested yet
*/

// set the id and can bus baud rate of the VESC
int can_baud = 500000;
int can_id = 90;

Thread vesc_thread;
Thread print_thread;
Thread control_thread;

CAN can1(PA_11, PA_12, can_baud); // define the can bus object & vesc object
vesc _vesc1;

void vesc_th() {
  _vesc1.vesc_init(&can1, can_baud);
  _vesc1.set_monitor_id(can_id); // This enable the can bus monitor particular
                                 // id, may improve later
  while (true) {
    _vesc1.can_read();
    printf("Read one msgs\r\n");
    ThisThread::sleep_for(10);
  }
}
void debug_msgs() {
  while (true) {
    // reading value
    printf("position: %d ", _vesc1.read_pos(can_id));
    printf("rpm: %d ", _vesc1.read_rpm(can_id));
    printf("current: %d ", _vesc1.read_current(can_id));
    printf("ESC current: %d ", _vesc1.read_esc_current(can_id));
    printf("input_voltage: %d ", _vesc1.read_input_voltage(can_id));
    printf("FET temp: %d \r\n", _vesc1.read_fet_temp(can_id));
    ThisThread::sleep_for(500);
  }
}
void control_cmd(){
    while(true){
        //_vesc1.set_curren(can_id, 3.0);
        printf("Send one msgs\r\n");
        ThisThread::sleep_for(100);
    }
}
int main() {
  // initslize the vesc
  vesc_thread.start(vesc_th);
  print_thread.start(debug_msgs);
  control_thread.start(control_cmd);
}
