
#ifndef ODriveMbed_h
#define ODriveMbed_h

#include "mbed.h"
#include <string>

class ODriveMbed {
public:
    enum AxisState_t {
        AXIS_STATE_UNDEFINED = 0,           //<! will fall through to idle
        AXIS_STATE_IDLE = 1,                //<! disable PWM and do nothing
        AXIS_STATE_STARTUP_SEQUENCE = 2, //<! the actual sequence is defined by the config.startup_... flags
        AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,   //<! run all calibration procedures, then idle
        AXIS_STATE_MOTOR_CALIBRATION = 4,   //<! run motor calibration
        AXIS_STATE_SENSORLESS_CONTROL = 5,  //<! run sensorless control
        AXIS_STATE_ENCODER_INDEX_SEARCH = 6, //<! run encoder index search
        AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7, //<! run encoder offset calibration
        AXIS_STATE_CLOSED_LOOP_CONTROL = 8  //<! run closed loop control
    };

    enum ControlMode_t{
        CTRL_MODE_VOLTAGE_CONTROL = 0,
        CTRL_MODE_CURRENT_CONTROL = 1,
        CTRL_MODE_VELOCITY_CONTROL = 2,
        CTRL_MODE_POSITION_CONTROL = 3,
        CTRL_MODE_TRAJECTORY_CONTROL = 4
    };
    
    ODriveMbed(Stream& serial);

    // Commands
    void setPosition(int motor_number, float position);
    void setPosition(int motor_number, float position, float velocity_feedforward);
    void setPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward);
    void setVelocity(int motor_number, float velocity);
    void setVelocity(int motor_number, float velocity, float current_feedforward);
    void setCurrent(int motor_number, float current);

    float getPositionEstimate(int axis);
    float getCurrentEstimate(int axis);

    // General params
    float readFloat();
    int32_t readInt();

    // Control Mode Helpers
    bool setControlMode(int axis, int requestedControlMode, bool readResult);
    int readControlMode(int axis);
    int readState(int axis);

    // State helper
    bool run_state(int axis, int requested_state, bool wait);

private:
    string readString();

    int _timeoutTime = 10; // in ms
    Stream& serial_;
};

#endif //ODriveMbed_h
