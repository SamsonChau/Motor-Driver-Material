
#include "MotorModule.h"


/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]

void pack_cmd(MotorStruct * motor){
     /// limit data to be within bounds ///
     motor->control.p_des = fminf(fmaxf(P_MIN, motor->control.p_des), P_MAX);                    
     motor->control.v_des = fminf(fmaxf(V_MIN, motor->control.v_des), V_MAX);
     motor->control.kp = fminf(fmaxf(KP_MIN, motor->control.kp), KP_MAX);
     motor->control.kd = fminf(fmaxf(KD_MIN, motor->control.kd), KD_MAX);
     motor->control.i_ff = fminf(fmaxf(I_MIN, motor->control.i_ff), I_MAX);
     /// convert floats to unsigned ints ///
     int p_int = float_to_uint(motor->control.p_des, P_MIN, P_MAX, 16);            
     int v_int = float_to_uint(motor->control.v_des, V_MIN, V_MAX, 12);
     int kp_int = float_to_uint(motor->control.kp, KP_MIN, KP_MAX, 12);
     int kd_int = float_to_uint(motor->control.kd, KD_MIN, KD_MAX, 12);
     int t_int = float_to_uint(motor->control.i_ff, I_MIN, I_MAX, 12);
     /// pack ints into the can buffer ///
     motor->txMsg.data[0] = p_int>>8;                                       
     motor->txMsg.data[1] = p_int&0xFF;
     motor->txMsg.data[2] = v_int>>4;
     motor->txMsg.data[3] = ((v_int&0xF)<<4)|(kp_int>>8);
     motor->txMsg.data[4] = kp_int&0xFF;
     motor->txMsg.data[5] = kd_int>>4;
     motor->txMsg.data[6] = ((kd_int&0xF)<<4)|(t_int>>8);
     motor->txMsg.data[7] = t_int&0xff;
     }
     
/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]

void unpack_reply(MotorStruct * motor){
    /// unpack ints from can buffer ///
    int id = motor->rxMsg.data[0];
    int p_int = (motor->rxMsg.data[1]<<8)|motor->rxMsg.data[2];
    int v_int = (motor->rxMsg.data[3]<<4)|(motor->rxMsg.data[4]>>4);
    int i_int = ((motor->rxMsg.data[4]&0xF)<<8)|motor->rxMsg.data[5];
    /// convert unsigned ints to floats ///
    float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    float i = uint_to_float(i_int, -I_MAX, I_MAX, 12);
    
    motor->state.position = p;
    motor->state.velocity = v;
    motor->state.current = i;  
    } 
    

void enable_motor(MotorStruct * motor, CAN * can)
{
    motor->txMsg.data[0] = 0xFF;
    motor->txMsg.data[1] = 0xFF;
    motor->txMsg.data[2] = 0xFF;
    motor->txMsg.data[3] = 0xFF;
    motor->txMsg.data[4] = 0xFF;
    motor->txMsg.data[5] = 0xFF;
    motor->txMsg.data[6] = 0xFF;
    motor->txMsg.data[7] = 0xFC;  
    can->write(motor->txMsg); 
}
void disable_motor(MotorStruct * motor, CAN * can)
{
    motor->txMsg.data[0] = 0xFF;
    motor->txMsg.data[1] = 0xFF;
    motor->txMsg.data[2] = 0xFF;
    motor->txMsg.data[3] = 0xFF;
    motor->txMsg.data[4] = 0xFF;
    motor->txMsg.data[5] = 0xFF;
    motor->txMsg.data[6] = 0xFF;
    motor->txMsg.data[7] = 0xFD;  
    can->write(motor->txMsg); 
}
