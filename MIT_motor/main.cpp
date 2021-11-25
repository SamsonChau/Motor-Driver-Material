
#define CAN_ID 0x0

#include "mbed.h"
#include "math_ops.h"
#include "MotorModule.h"



Serial       pc(PA_2, PA_3);                // Serial port to the computer
CAN          can(PB_8, PB_9, 1000000);      // CAN Rx pin name, CAN Tx pin name

Ticker loop;                                // Control loop interrupt handler
int loop_counter = 0;
#define DT                  .01f             // Control loop period

#define N_MOTORS             2              // Number of motors on the can bus
MotorStruct motors[N_MOTORS];               // Create a list of the motors attached

/* Communication functions.  Do not touch */
void onMsgReceived();
void init_motors(int ids[N_MOTORS]);
/*                                       */


void control()
{
    /* Your control loop goes here.  */
    /* Update torques, position/velocity setpoints, etc */
    //motors[0].control.p_des = -40 + 40.0f*cos(.01f*loop_counter);

    float tilt_angle = 1.35f;
    
    float t = DT*loop_counter;
    if(t<1)
    {
        motors[1].control.p_des = -tilt_angle*t; // dump left to -1.5
        motors[0].control.p_des = 0;
    }   
    else if(t>1 && t<3)
    {
        motors[1].control.p_des = tilt_angle*(t-1.0f) - tilt_angle; // dump right from -1.5 to 1.5
        motors[0].control.p_des = 0;
    }   
    else if(t>3 && t<4)
    {
        motors[1].control.p_des = -tilt_angle*(t-3.0f) + tilt_angle;
        motors[0].control.p_des = 0;
    }   // center from 1.5 to 0
    else if (t>4 && t<7)
    {
        motors[1].control.p_des = 0;
        motors[0].control.p_des = -40.0f+40.0f*cos((t-4.0f));
    }
    else if(t>7 && t<8)
    {
        motors[1].control.p_des = -tilt_angle*(t-7.0f); // dump left to -1.5
        motors[0].control.p_des = -80;
    }   
    else if(t>8 && t<10)
    {
        motors[1].control.p_des = tilt_angle*(t-8.0f) - tilt_angle; // dump right from -1.5 to 1.5
        motors[0].control.p_des = -80;
    }   
    else if(t>10 && t<11)
    {
        motors[1].control.p_des = -tilt_angle*(t-10.0f) + tilt_angle;
        motors[0].control.p_des = -80;
    }   // center from 1.5 to 0
    else if (t>11 && t<14)
    {
        motors[1].control.p_des = 0;
        motors[0].control.p_des = -40.0f-40.0f*cos((t-11.0f));
    }
    
    motors[0].control.kd = .5f;
    motors[0].control.kp = 2.0f;
    
    //motors[1].control.p_des = 2*sin(.01f*loop_counter);
    motors[1].control.kd = 1.0f;
    motors[1].control.kp = 20.0f;
    /*                              */
    
    if(t>14){loop_counter = 0;}
    
    for(int i = 0; i<N_MOTORS; i++)
    {
        pack_cmd(&motors[i]);
        can.write(motors[i].txMsg);
    }
    
    
    //printf("%f  %f\n\r", motors[0].control.p_des, motors[1].control.p_des);           // This will print to the computer.  Usefull for debugging
    printf("%f  %f\n\r", motors[0].state.position, motors[1].state.position);
    loop_counter++;     // Increment loop counter
}


int main() 
{
    
    pc.baud(921600);                            // Set baud rate for communication over USB serial
    can.attach(&onMsgReceived);                 // attach 'CAN receive-complete' interrupt handler
    can.filter(CAN_ID , 0xFFF, CANStandard, 0); // Set up can filter so it interrups only for messages with ID CAN_ID
    
    int ids[N_MOTORS] = {1, 2};                 // List of motor CAN ID's
    init_motors(ids);                           // Initialize the list of motors
    
    enable_motor(&motors[0], &can);             // Enable first motor
    enable_motor(&motors[1], &can);
    wait(1);                                    // Wait 1 second
    //disable_motor(&motors[0], &can);            // Disable first motor
    //disable_motor(&motors[1], &can); 
    loop.attach(&control, DT);                 // Start running the contorl interrupt at 1/DT Hz
        
    while(1) 
    {
        // Usuallly nothing should run here.  Instead run control in the interrupt.
    }
        
}
    

/* low-level communication functoins below.  Do not touch */


 void onMsgReceived() 
/* This interrupt gets called when a CAN message with ID CAN_ID shows up */
{
    CANMessage   rxMsg;
    rxMsg.len = 6;
    can.read(rxMsg);                    // read message into Rx message storage
    int id = rxMsg.data[0];
    for (int i = 0; i< N_MOTORS; i++)
    {
        if(motors[i].control.id == id)
        {
            memcpy(&motors[i].rxMsg, &rxMsg, sizeof(motors[i].rxMsg));
            unpack_reply(&motors[i]);
        }
    }
}

void init_motors(int ids[N_MOTORS])
/* Initialize buffer lengths and IDs of the motors in the list */
{
    for(int i = 0; i<N_MOTORS; i++)
    {
        motors[i].txMsg.len = 8;
        motors[i].rxMsg.len = 6;
        motors[i].control.id = ids[i];
        motors[i].txMsg.id = ids[i];
        motors[i].control.p_des = 0;
        motors[i].control.v_des = 0;
        motors[i].control.kp = 0;
        motors[i].control.kd = 0;
        motors[i].control.i_ff = 0;
    }
}
