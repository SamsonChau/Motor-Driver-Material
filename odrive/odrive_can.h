#include "mbed.h"
#include <cstdint>

#define can_nmt_cmd 0x000 
#define odrive_heartbeat_cmd  0x001
#define odrive_estop_cmd 0x002
#define reboot_odrive_cmd 0x016  
#define clear_error_flags_cmd 0x018 
#define heartbeat_msg 0x700 
#define start_anticogging_cmd 0x010

#define set_node_id_cmd 0x006
#define set_controller_mode_cmd 0x00B 
#define set_input_pos_cmd 0x00C  
#define set_input_vel_cmd 0x00D 
#define set_input_torque_cmd 0x00E 
#define set_limit_cmd 0x00F 
#define set_linear_count_cmd 0x019 
#define set_position_gain_cmd 0x01A  
#define set_vel_gain_cmd 0x01B  
#define set_traj_vel_limit_cmd 0x011 
#define set_traj_acc_limit_cmd 0x012  
#define set_traj_intertia_cmd 0x013 
#define set_axis_request_state_cmd 0x007 
#define set_axis_startup_config_cmd 0x008

#define get_motor_error_cmd 0x003
#define get_encoder_error_cmd 0x004
#define get_sensorless_error_cmd 0x005
#define get_IQ_cmd 0x014
#define get_sensorless_estimate_cmd 0x015
#define get_encoder_esitimates_cmd 0x009 
#define get_encoder_count_cmd 0x00A 
#define get_vbus_voltage_cmd 0x017 

class odrive_can{
    public:
        int8_t node_id = 0x00; //from 0 (0x00) to 63 (0x3F)
        void odrive_init(CAN* _CAN);

        void set_node_id(int8_t new_id);
        void set_controller_mode(int32_t control_mode, int32_t input_mode);
        void reboot();
        void scan_network();
        bool estop();
        bool checkstate();
        void clear_error_flags();

        void set_pos(float pos, int16_t v_ff, int16_t t_ff);
        void set_vel(float vel,int16_t t_ff);
        void set_toque(float torque);
        void set_limit(float vel_limit, float current_limit);

        void set_traj_vel_limit(float traj_vel_limt);
        void set_traj_acc_limit(float traj_acc_limit);
        void set_traj_intertia(float traj_inertia);

        void set_anticogging();
        void set_axis_startup_config();
        void set_axis_request_state(uint32_t state);

        void set_linear_count(int32_t cnt);
        void set_position_gain(float pos_gain);
        void set_vel_gain(float vel_gain, float vel_i_gain);
        
        int32_t get_encoder_count(int8_t mode);
        int32_t get_vbus_voltage();
        int32_t get_sensorless_estimate(int8_t mode);
        int32_t get_encoder_esitimates(int8_t mode);
        int32_t get_IQ();
        uint64_t get_motor_error();
        uint32_t get_encoder_error();
        uint32_t get_sensorless_error();

        CAN* can1;

        int32_t encoder_cnt = 0;        // Encoder Count in CPR
        int32_t encoder_shadow_cnt = 0; // Encodr Shadow Count
        int32_t encoder_pos_est = 0;    // Encoder Position Estimate
        int32_t encoder_vel_est = 0;    // Encoder Velocity Estimate
        uint32_t encoder_error = 0;     // Encoder ERROR code
        
        int32_t sensorless_pos_est = 0; // Sensorless Position Estimate
        int32_t sensorless_vel_est = 0; // Sensorless Velocity Estimate
        uint32_t sensorless_error = 0;  // Sensorless ERROR code
        
        int32_t vbus = 0;               // Vbus Voltage 
        int32_t Iq_set_pt = 0;          // Iq constant set point
        int32_t Iq_measure = 0;         // Iq constant measured

        uint64_t motor_error = 0;       // Motor ERROR code
        uint32_t axis_error = 0;         // Axis ERROR state / ERROR code
        uint8_t axis_current_state = 0;  // Axis current state
        uint8_t axis_controller_state = 0;//Axis controller state 

    private:
        bool can_send_data(int16_t cmd, int8_t data[8], int8_t framesize);
        bool can_send_cmd(int16_t cmd);
        int can_read();
};
