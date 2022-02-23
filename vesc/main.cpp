/////////////////////////////////////////////////////////////////////////////
/*
    VESC libraray sample program, 
    Use mbed os 6.13 to compile.
    connect the vesc with the STM32 via can bus
    config the vesc in vesc tools before connect to the VESC
    change the setting according your need
    
    ALL cmd are usingg the same unit as vesc
*/
/////////////////////////////////////////////////////////////////////////////
#include "mbed.h"
#include "vesc.h"

/////////////////////////////////////////////////////////////////////////////
/*
      USB serial config 
      for show the debug msg and display the 
*/
////////////////////////////////////////////////////////////////////////////

#define TARGET_TX_PIN USBTX
#define TARGET_RX_PIN USBRX
static BufferedSerial serial_port(TARGET_TX_PIN, TARGET_RX_PIN, 115200);


/////////////////////////////////////////////////////////////////////////////
/*
      CAN BUS config 
      set up the VESC can bus communication channel of the can bus 
      Create one object for one VESC ID
      if the read status is not work you may concider to slower the baudrate 
      and the send status msgs frequency of the vesc
      for one channel you can connect up to 10 VSEC or more(not tested more than 10)
      
      Hardware controller: Nucleo STM32F446RE
      CAN-BUS pinout: can1 (PA_11, PA_12), can2 (PB_5, PB_6) 
      example id: 90
      example can bus baud rate: 1Mbps (MAX)
      example vesc send status msgs frequency: 3000Hz (MAX) 
*/
////////////////////////////////////////////////////////////////////////////
// set the id and can bus baud rate of the VESC
int can_baud = 1000000;
int can_id = 90;

// define the can bus object
// CAN can1(PA_11, PA_12, can_baud);  (choose one)
CAN can(PB_5, PB_6, can_baud);    

vesc _vesc1;

Thread vesc_thread(osPriorityNormal);
Thread control_thread(osPriorityLow);

EventQueue printfQueue;
EventQueue readQueue;

Ticker ThreadTicker;
Ticker printreadTicker;
                                                                                                                                                                                                                                                                                                                                


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
