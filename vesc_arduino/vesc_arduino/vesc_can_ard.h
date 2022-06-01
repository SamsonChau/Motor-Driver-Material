#include <SPI.h>
#include <mcp2515.h>
#include "vesc_can_id.h"

#define vesc_serial_debug true
// internal storage
volatile int current_id;
volatile float pid_pos;
volatile float rpm;
volatile float current;
volatile float esc_current;
volatile float input_voltage;
volatile float fet_temp;
volatile bool interrupt;

MCP2515 mcp2515(7);

void irqHandler() {
  interrupt = true;
}

//intialize the class vesc, can bus setting the can bus message type
void vesc_init() {
  //CAN
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  attachInterrupt(0, irqHandler, FALLING);
}
//reset the parameter of the internal class
void reset_param() {
  current_id = 0;
  pid_pos = 0;
  rpm = 0;
  current = 0;
  esc_current = 0;
  input_voltage = 0;
  fet_temp = 0;
}
//set up the which id you wanna monitor to
void set_monitor_id(int id) {
  current_id = id;
  if (current_id != id) {
    reset_param();
  }
}
//internal function to format the data in to canbus msg
void package_msg(uint8_t *buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}
//Send the msg to the canbus from the buffer packet
bool can_send(int id, uint8_t packet[], int32_t len) {
  struct can_frame Txmsg;
  Txmsg.can_id = id ;//| CAN_EFF_FLAG;
  Txmsg.can_dlc = sizeof(packet);
  for (int i = 0; i < sizeof(packet); i++) {
    Txmsg.data[i] = packet[i];
  }
  if(vesc_serial_debug){
    Serial.print("TX_ID: 0x");
    Serial.print(Txmsg.can_id);
    Serial.print(" ID: 0x");
    Serial.print(id, HEX);
    Serial.print("CANID: ");
    Serial.print(Txmsg.can_id & 0xFF );
    Serial.print(" CMD: ");
    Serial.print(id >>8);
    Serial.print("\n");
  }
  mcp2515.sendMessage(MCP2515::TXB1, &Txmsg);
  return true;
}


// Set vesc command

// duty cycle setup cmd
// set up the duty cycle of the vec in %, simular to the PWM control
// same as the vesc tools
// value -100 -> 0 -> 100
void set_duty(int id, float duty) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  package_msg(buffer, (int32_t)(duty * 100000.0), &send_index);
  can_send((uint32_t)id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

// current control cmd
// set up the drive current of the vesc in A
// same as the vesc tools
// value -60 0 -> 60 (physical limit, may vary from different current setting)
void set_current(int id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  package_msg(buffer, (int32_t)(current * 1000.0), &send_index);
  can_send(id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

// braking current cmd
// set up the current for braking in A
// same as vesc tool
// value 0 (if you didnot config it)
void set_current_brake(int id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  package_msg(buffer, (int32_t)(current * 1000.0), &send_index);
  can_send(id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer,
           send_index);
}

// velocity control cmd
// set up the velocity pid control set point in rpm/erpm
// same as vesc tools
// for Sersorless motor the rpm means the erpm of the motor
// for Hall Sensor/ Encoder motor this means the rpm read form the encoder
// VESC6 (HW) erpm limit +-150000
// VESC4 (HW) erpm limit +-60000
void set_rpm(int id, float rpm) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  package_msg(buffer, (int32_t)rpm, &send_index);
  can_send(id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

// position control cmd
// set up the position pid control set point in deg
// same as vesc tools
// for Sersorless motor not recommand to use this function
// for Hall Sensor/ Encoder motor this means the position read form the encoder
void set_pos(int id, float pos) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  package_msg(buffer, (int32_t)(pos * 1000000.0), &send_index);
  can_send(id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

// read vesc command
// decode the vesc can bus command
// reference to the xlsx file given
void can_read(int id) {
  if (interrupt) {
    interrupt = false;
    uint8_t irq = mcp2515.getInterrupts();
    struct can_frame Rxmsg;
    if (irq & MCP2515::CANINTF_RX0IF) {
      if (mcp2515.readMessage(MCP2515::RXB0, &Rxmsg) == MCP2515::ERROR_OK) {
        int can_id = Rxmsg.can_id & 0xFF;
        int cmd = (Rxmsg.can_id >> 8);
        if (can_id == id) {
          switch (cmd) {
            case CAN_PACKET_STATUS:
              rpm = (float)((int32_t)(((uint32_t)Rxmsg.data[0]) << 24 |
                                      ((uint32_t)Rxmsg.data[1]) << 16 |
                                      ((uint32_t)Rxmsg.data[2]) << 8 | ((uint32_t)Rxmsg.data[3])));
              esc_current =
                (float)(((uint16_t)Rxmsg.data[4]) << 8 | ((uint16_t)Rxmsg.data[5])) /
                10.0;
              break;

            case CAN_PACKET_STATUS_4:
              fet_temp =
                (float)(((uint16_t)Rxmsg.data[0]) << 8 | ((uint16_t)Rxmsg.data[1])) /
                10.0;
              current =
                (float)(((uint16_t)Rxmsg.data[4]) << 8 | ((uint16_t)Rxmsg.data[5])) /
                10.0;
              pid_pos =
                (float)(((uint16_t)Rxmsg.data[6]) << 8 | ((uint16_t)Rxmsg.data[7])) /
                50.0;
              break;

            case CAN_PACKET_STATUS_5:
              input_voltage =
                (float)(((uint16_t)Rxmsg.data[4]) << 8 | ((uint16_t)Rxmsg.data[5])) /
                10.0;
              break;
            default:
              break;
          }
        }
      }
    }
    if (irq & MCP2515::CANINTF_RX1IF) {
      if (mcp2515.readMessage(MCP2515::RXB1, &Rxmsg) == MCP2515::ERROR_OK) {
        int can_id = Rxmsg.can_id & 0xFF;
        int cmd = (Rxmsg.can_id >> 8);
        if (can_id == id) {
          switch (cmd) {
            case CAN_PACKET_STATUS:
              rpm = (float)((int32_t)(((uint32_t)Rxmsg.data[0]) << 24 |
                                      ((uint32_t)Rxmsg.data[1]) << 16 |
                                      ((uint32_t)Rxmsg.data[2]) << 8 | ((uint32_t)Rxmsg.data[3])));
              esc_current =
                (float)(((uint16_t)Rxmsg.data[4]) << 8 | ((uint16_t)Rxmsg.data[5])) /
                10.0;
              break;

            case CAN_PACKET_STATUS_4:
              fet_temp =
                (float)(((uint16_t)Rxmsg.data[0]) << 8 | ((uint16_t)Rxmsg.data[1])) /
                10.0;
              current =
                (float)(((uint16_t)Rxmsg.data[4]) << 8 | ((uint16_t)Rxmsg.data[5])) /
                10.0;
              pid_pos =
                (float)(((uint16_t)Rxmsg.data[6]) << 8 | ((uint16_t)Rxmsg.data[7])) /
                50.0;
              break;

            case CAN_PACKET_STATUS_5:
              input_voltage =
                (float)(((uint16_t)Rxmsg.data[4]) << 8 | ((uint16_t)Rxmsg.data[5])) /
                10.0;
              break;
            default:
              break;
          }
        }
      }
    }
  }
}
// the position (in deg) provide by the vesc in deg
// will return a float32
float read_pos(int id) {
  if (id != current_id) {
    current_id = id;
    reset_param();
    // printf("updated monitor esc id");
    return 0;
  } else {
    return pid_pos;
  }
}
// the rpm provide by the vesc in rpm
// will return a float32
float read_rpm(int id) {
  if (id != current_id) {
    current_id = id;
    reset_param();
    // printf("updated monitor esc id");
    return 0;
  } else {
    return rpm;
  }
}
// output current provide by the vesc in A
// will return a float32
float read_current(int id) {
  if (id != current_id) {
    current_id = id;
    reset_param();
    // printf("updated monitor esc id");
    return 0;
  } else {
    return esc_current;
  }
}
// the input bus current provide by the vesc in A
// will return a float32
float read_esc_current(int id) {
  if (id != current_id) {
    current_id = id;
    reset_param();
    // printf("updated monitor esc id");
    return 0;
  } else {
    return esc_current;
  }
}
// the input bus voltage provide by the vesc in V
// will return a float32
float read_input_voltage(int id) {
  if (id != current_id) {
    current_id = id;
    reset_param();
    // printf("updated monitor esc id");
    return 0;
  } else {
    return input_voltage;
  }
}
// the Mosfet temperture provided by vesc in deg (C)
// will return a float32
float read_fet_temp(int id) {
  if (id != current_id) {
    current_id = id;
    reset_param();
    // printf("updated monitor esc id");
    return 0;
  } else {
    return fet_temp;
  }
}
