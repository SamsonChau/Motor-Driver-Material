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
CAN can(PB_5, PB_6, can_baud);              // define thhe can bus as can_Tx_pin can_Rx_pin can baudrate

//define the vesc object of the vesc
vesc _vesc1;


/////////////////////////////////////////////////////////////////////////////
/*
        RTOS stuff
        not recommanded for junior
        This example have 2 thread
        1. vesc_thread (read the vsec can status in 1000Hz) trigger by ThreadTicker
        2. control_thread (send the vesc control cmd and the UART debug message in 20Hz)
*/
////////////////////////////////////////////////////////////////////////////
Thread vesc_thread(osPriorityNormal);       // define a thread in Normal Priorty
Thread control_thread(osPriorityLow);       // define a thread in Low Priorty 

EventQueue printfQueue;                     // define a Quenue to pipeline the Thread
EventQueue readQueue;                       // define a Quenue to pipeline the Thread

Ticker ThreadTicker;                        // define a Ticker(SysTick) as a trigger of the Thread
Ticker printreadTicker;                     // define a Ticker(SysTick) as a trigger of the Thread
                                                                                                                                                                                                                                                                                                                                
//call back of the vesc thread function 
void vesc_th() { 
    // update the vesc1 parameter once 
    _vesc1.can_read(can_id); 
}

//call back of the debug thread function 
void debug_msgs() {
  // print out the values
  printf("position: %.2f ", _vesc1.read_pos(can_id));
  printf("rpm: %.2f ", _vesc1.read_rpm(can_id));
  printf("current: %.2f ", _vesc1.read_current(can_id));
  printf("ESC current: %.2f ", _vesc1.read_esc_current(can_id));
  printf("input_voltage: %.2f ", _vesc1.read_input_voltage(can_id));
  printf("FET temp: %.2f \r\n", _vesc1.read_fet_temp(can_id));
}
void control_cmd() {
  //_vesc1.set_rpm(can_id, 1000);         // set the vesc 1 erpm/rpm to 1000rpm
  printfQueue.call(&debug_msgs);          // call the debug_msgs to run
}
int main() {
  // intitial the vesc class at startup, need to pass the canbus interface to the init  function
  _vesc1.vesc_init(&can, can_baud);
  _vesc1.set_monitor_id(can_id);        //set up the moitoring ID of the VESC object
  
  // start the threads and run forever
  vesc_thread.start(callback(&readQueue, &EventQueue::dispatch_forever));
  control_thread.start(callback(&printfQueue, &EventQueue::dispatch_forever));
    
  //setup the trigger of the thread, use Tickers to achieve
  printreadTicker.attach(readQueue.event(&vesc_th), 0.001);     // trigger once after 0.001 sec 
  ThreadTicker.attach(printfQueue.event(&control_cmd), 0.05);   // trigger once after 0.05 sec
}

/* standalone version
#include "mbed.h"
#include "vesc.h"

#define TARGET_TX_PIN USBTX
#define TARGET_RX_PIN USBRX
static BufferedSerial serial_port(TARGET_TX_PIN, TARGET_RX_PIN, 115200);

// set the id and can bus baud rate of the VESC
int can_baud = 1000000;
int can_id = 90;

// define the can bus object
// CAN can1(PA_11, PA_12, can_baud);  (choose one)
CAN can(PB_5, PB_6,can_baud); // define thhe can bus as can_Tx_pin can_Rx_pin can baudrate

// define the vesc object of the vesc
vesc _vesc1;

Ticker readTicker;

// call back of the vesc thread function
void vesc_th() {
  // update the vesc1 parameter once
  _vesc1.can_read(can_id);
}

int main() {
  // intitial the vesc class at startup, need to pass the canbus interface to
  // the init  function
  _vesc1.vesc_init(&can, can_baud);
  _vesc1.set_monitor_id(can_id); // set up the moitoring ID of the VESC object
  
  // setup the trigger of the thread, use Tickers to achieve
  readTicker.attach(&vesc_th, 0.001); // trigger once after 0.001 sec
  
  while (true) {
    //_vesc1.set_rpm(can_id, 1000);         // set the vesc 1 erpm/rpm to 1000rpm
    // print out the values
    printf("position: %.2f ", _vesc1.read_pos(can_id));
    printf("rpm: %.2f ", _vesc1.read_rpm(can_id));
    printf("current: %.2f ", _vesc1.read_current(can_id));
    printf("ESC current: %.2f ", _vesc1.read_esc_current(can_id));
    printf("input_voltage: %.2f ", _vesc1.read_input_voltage(can_id));
    printf("FET temp: %.2f \r\n", _vesc1.read_fet_temp(can_id));

    ThisThread::sleep_for(10ms);
  }
}
*/
