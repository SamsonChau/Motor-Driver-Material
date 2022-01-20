#include "mbed.h"
#include "vesc_id.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define UTILS_LP_FAST(value, sample, filter_constant)   (value -= (filter_constant) * (value - (sample)))

class vesc
{
public:
    void vesc_init(CAN* _CAN, int baud);
    void set_monitor_id(uint8_t id);

    void set_duty(uint8_t id,float duty);
    void set_current(uint8_t id, float current);
    void set_current_brake(uint8_t id, float current);
    void set_rpm(uint8_t id, float rpm);
    void set_pos(uint8_t id, float pos);

    int16_t read_pos(uint8_t id);
    int32_t read_rpm(uint8_t id);
    int16_t read_current(uint8_t id);
    int16_t read_esc_current(uint8_t id);
    int16_t read_input_voltage(uint8_t id);
    int16_t read_fet_temp(uint8_t id);

private:
    void package_msg(uint8_t* buffer, int32_t number, int32_t *index);
    bool can_send(uint32_t id, uint8_t packet[], int32_t len);
    void can_read();

    void reset_param();

    uint8_t current_id;
    int16_t pid_pos;
    int32_t rpm;
    int16_t current;
    int16_t esc_current;
    int16_t input_voltage;
    int16_t fet_temp;

    CAN* can;
    CANMessage Txmsg;
    CANMessage Rxmsg;
    unsigned char len = 0;
};