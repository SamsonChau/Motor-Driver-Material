#include "mbed.h"
#include "vesc.h"

#define TARGET_TX_PIN USBTX
#define TARGET_RX_PIN USBRX
static BufferedSerial serial_port(TARGET_TX_PIN, TARGET_RX_PIN, 115200);
/*
config the vesc in vesc tools before testing
change the setting according your need
the read status function not tested yet
*/

// set the id and can bus baud rate of the VESC
int can_baud = 1000000;
int can_id = 90;

// CAN can1(PA_11, PA_12, can_baud); // define the can bus object & vesc object
CAN can(PB_5, PB_6, can_baud);    

Thread vesc_thread(osPriorityNormal);
Thread control_thread(osPriorityLow);

EventQueue printfQueue;
EventQueue readQueue;

Ticker ThreadTicker;
Ticker printreadTicker;
                                                                                                                                                                                                                                                                                                                                
vesc _vesc1;

void vesc_th() { _vesc1.can_read(can_id); }

void debug_msgs() {
  // reading value
  printf("position: %.2f ", _vesc1.read_pos(can_id));
  printf("rpm: %.2f ", _vesc1.read_rpm(can_id));
  printf("current: %.2f ", _vesc1.read_current(can_id));
  printf("ESC current: %.2f ", _vesc1.read_esc_current(can_id));
  printf("input_voltage: %.2f ", _vesc1.read_input_voltage(can_id));
  printf("FET temp: %.2f \r\n", _vesc1.read_fet_temp(can_id));
}
void control_cmd() {
  //_vesc1.set_rpm(can_id, 10000);
  //printf("Send one msgs\r\n");
  printfQueue.call(&debug_msgs);
}
int main() {
  _vesc1.vesc_init(&can, can_baud);
  _vesc1.set_monitor_id(can_id); // This enable the can bus monitor particular
                                 // id, may improve later

  // initslize the vesc
  vesc_thread.start(callback(&readQueue, &EventQueue::dispatch_forever));
  control_thread.start(callback(&printfQueue, &EventQueue::dispatch_forever));
  
  printreadTicker.attach(readQueue.event(&vesc_th), 0.001);
  ThreadTicker.attach(printfQueue.event(&control_cmd), 0.05);
}
