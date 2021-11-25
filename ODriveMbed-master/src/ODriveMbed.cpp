
#include "ODriveMbed.h"

static const int kMotorOffsetFloat = 2;
static const int kMotorStrideFloat = 28;
static const int kMotorOffsetInt32 = 0;
static const int kMotorStrideInt32 = 4;
static const int kMotorOffsetBool = 0;
static const int kMotorStrideBool = 4;
static const int kMotorOffsetUint16 = 0;
static const int kMotorStrideUint16 = 2;


ODriveMbed::ODriveMbed(Stream& serial) 
    : serial_(serial) {}

void ODriveMbed::setPosition(int motor_number, float position) {
    setPosition(motor_number, position, 0.0f, 0.0f);
}

void ODriveMbed::setPosition(int motor_number, float position, float velocity_feedforward) {
    setPosition(motor_number, position, velocity_feedforward, 0.0f);
}

void ODriveMbed::setPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward) {
    //serial_ << "p " << motor_number  << " " << position << " " << velocity_feedforward << " " << current_feedforward << "\n";
    serial_.printf("p %d %f %f %f\n", motor_number, position, velocity_feedforward, current_feedforward);
}

void ODriveMbed::setVelocity(int motor_number, float velocity) {
    setVelocity(motor_number, velocity, 0.0f);
}

void ODriveMbed::setVelocity(int motor_number, float velocity, float current_feedforward) {
    //serial_ << "v " << motor_number  << " " << velocity << " " << current_feedforward << "\n";
    serial_.printf("v %d %f %f\n", motor_number, velocity, current_feedforward);
}

void ODriveMbed::setCurrent(int motor_number, float current){
    serial_.printf("c %d %f\n", motor_number, current);
}


float ODriveMbed::getPositionEstimate(int axis){
    int timeout_ctr = 5;
    float position;
        do {
            wait_ms(_timeoutTime);
            //serial_ << "r axis" << axis << ".current_state\n";
            serial_.printf("r axis%d.encoder.pos_estimate\n",axis);
            position = readFloat();
        } while (--timeout_ctr > 0);
    return position;
}

float ODriveMbed::getCurrentEstimate(int axis){
    int timeout_ctr = 5;
    float current;
        do {
            wait_ms(_timeoutTime);
            //serial_ << "r axis" << axis << ".current_state\n";
            serial_.printf("r axis%d.motor.current_control.Iq_measured\n",axis);
            current = readFloat();
        } while (--timeout_ctr > 0);
    return current;
}


float ODriveMbed::readFloat() {
    return (float)atof(readString().c_str());
}

int32_t ODriveMbed::readInt() {
    return atoi(readString().c_str());
}

bool ODriveMbed::run_state(int axis, int requested_state, bool read_) {
    int timeout_ctr = 100;
    //serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';
    serial_.printf("w axis%d.requested_state %d\n", axis, requested_state);
    if (read_) {
        do {
            wait_ms(_timeoutTime);
            //serial_ << "r axis" << axis << ".current_state\n";
            serial_.printf("r axis%d.current_state\n",axis);
        } while (readInt() != AXIS_STATE_IDLE && --timeout_ctr > 0);
    }

    return timeout_ctr > 0;
}

bool ODriveMbed::setControlMode(int axis, int requestedControlMode, bool read_){
    int timeout_ctr = 100;
    //serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';
    serial_.printf("w axis%d.controller.config.control_mode %d\n", axis, requestedControlMode);
    if (read_) {
        do {
            wait_ms(_timeoutTime);
            //serial_ << "r axis" << axis << ".current_state\n";
            serial_.printf("r axis%d.controller.config.control_mode\n",axis);
            
        } while (readInt() != requestedControlMode && --timeout_ctr > 0);
    }

    return timeout_ctr > 0;
}

int ODriveMbed::readControlMode(int axis){
    serial_.printf("r axis%d.controller.config.control_mode\n",axis);
    return readInt();
}

int ODriveMbed::readState(int axis){
    int timeout_ctr = 5;
    int state = -1;
        do {
            wait_ms(_timeoutTime);
            //serial_ << "r axis" << axis << ".current_state\n";
            serial_.printf("r axis%d.current_state\n",axis);
            state = readInt();
        } while (--timeout_ctr > 0);
    return state;
}

string ODriveMbed::readString() {
    string str = "";
    static const unsigned int timeout = 1000; // Value in milli-seconds
    Timer t;
    t.start();
    unsigned int timeout_start = t.read_ms();
    for (;;) {
        while (!serial_.readable()) {
            if (t.read_ms() - timeout_start >= timeout) {
                return str;
            }
        }
        char c = serial_.getc();
        if (c == '\n')
            break;
        str += c;
    }
    t.stop();
    return str;
}
