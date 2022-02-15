# Mbed OS lib for the vesc can bus communication
Sample program and library to communicate VESC and STM32 via can bus.use mbed os 6.13 to compile. 

## Feature 
* Added the readed Function
* update rate up to 1000Hz
* Update the command list

## Improvement
* Can read state of different id vesc in one vesc object
* reduce size of the program
* add can network search function 

## Installation
1. copy all the file to your Mbed Studio
2. set the can bus pinout and target board
3. config the vesc from VESC Tools
4. Test it by sending the random command to the vesc via canbus

## VESC tool setting
VESC tools version: 4.00
![](https://github.com/SamsonChau/Motor-Driver-Material/blob/main/vesc/Doc/vesc_can_setting.png)
![](https://github.com/SamsonChau/Motor-Driver-Material/blob/main/vesc/Doc/vesc_remote_setting.png)

## User Function
```cpp
    void vesc_init(CAN* _CAN, int baud);                  //init the object  
    void set_monitor_id(int id);                          //set the read status vesc id, currently only support one only

    void set_duty(int id,float duty);                     //set the duty cycle set point for the motor
    void set_current(int id, float current);              //set the current set point for the motor
    void set_current_brake(int id, float current);        //set the braking current of the motor
    void set_rpm(int id, float rpm);                      //set the velocity of the motor
    void set_pos(int id, float pos);                      //set the position of the motor

    float read_pos(int id);                               //read the pid position feedback 
    float read_rpm(int id);                               //read the erpm of the motor
    float read_current(int id);                           //read the current to the motor
    float read_esc_current(int id);                       //read the input bus current
    float read_input_voltage(int id);                     //read the input voltage of the bus
    float read_fet_temp(int id);                          //read the motor tempeture of the FET
```


