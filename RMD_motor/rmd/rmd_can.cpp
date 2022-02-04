#include "rmd_can.h"

void rmd_can::rmd_can_init(CAN *_CAN) {
  can1 = _CAN;
  can1->frequency(1000000);
  id = 1;
  data_buff[8] = {0};
  temp = 0;
  voltage = 0;
  error = 0;
  p.kp, p.ki = 0;
  v.kp, v.ki = 0;
  i.kp, i.ki = 0;
  motor_disable(id);
}
int8_t rmd_can::can_send(int16_t id, int8_t data[]) {
  Tx_msg.id = Header_ID + id;
  Tx_msg.format = CANStandard;
  Tx_msg.type = CANData;
  Tx_msg.len = 8;
  Tx_msg.data[0] = data[0];
  Tx_msg.data[1] = data[1];
  Tx_msg.data[2] = data[2];
  Tx_msg.data[3] = data[3];
  Tx_msg.data[4] = data[4];
  Tx_msg.data[5] = data[5];
  Tx_msg.data[6] = data[6];
  Tx_msg.data[7] = data[7];

  int cnt = 0;
  while (!can1->write(Tx_msg) && cnt < 3) {
    if (DEBUG_RMD_LIB_MSG) {
      printf("sent command fail! retry %d\r\n", cnt);
    }
    cnt++;
  }

  if (can1->write(Tx_msg)) {
    if (DEBUG_RMD_LIB_MSG) {
      printf("Sent cmd!\r\n");
    }
    cnt = 0;
    return 1;
  } else {
    if (DEBUG_RMD_LIB_MSG) {
      printf("sent command fail!\r\n");
    }
    cnt = 0;
    return -1;
  }
  cnt = 0;
  return 0;
}

uint8_t rmd_can::can_read(int16_t id, uint8_t cmd) {
  int cnt = 0;

  while (cnt < 4) {
    // read the reply 4 times
    can1->read(Rx_msg);
    if (DEBUG_RMD_LIB_MSG) {
      printf("RX msg receieved!\r\n");
      printf("msg ID: 0x%x\r\n", Rx_msg.id);
      printf("cmd ID: 0x%x\r\n", Rx_msg.data[0]);
      printf("msg read attempt: %d\r\n", cnt);
    }
    if (Rx_msg.data[0] == cmd) {
      if (DEBUG_RMD_LIB_MSG) {
        printf("RX msg receieved!\r\n");
        printf("msg ID: 0x%x\r\n", Rx_msg.id);
      }
      // check device reply id
      if (Rx_msg.id != Header_ID+id) {
        if (DEBUG_RMD_LIB_MSG) {
          printf("Device ID error\r\n!");
        }
        return -1;
      } 
      else if (Rx_msg.data[0] != cmd) {
        if (DEBUG_RMD_LIB_MSG) {
          printf("cmd error\r\n!");
        }
        return -1;
      } else {
        // load data to buffer
        for (int i = 0; i < Rx_msg.len; i++) {
          data_buff[i] = Rx_msg.data[i];
        }
        // return the command ID
        return cmd;
      }
    }
    cnt++;
  }
  if (cnt > 4) {
    if (DEBUG_RMD_LIB_MSG) {
      printf("attempt read overflow\r\n");
    }
    return -1;
  }
  return 0;
}

void rmd_can::read_pid(int8_t id) {
  int8_t cmd[8] = {};
  cmd[0] = READ_PID_ID;
  if (can_send(id, cmd)) {
    if (can_read(id, READ_PID_ID) == READ_PID_ID) {
      p.kp = data_buff[2];
      p.ki = data_buff[3];
      v.kp = data_buff[4];
      v.ki = data_buff[5];
      i.kp = data_buff[6];
      i.ki = data_buff[7];
      if (DEBUG_RMD_LIB_MSG) {
        printf("PID param updated!\r\n");
      }
      // return 0;
    } else {
      if (DEBUG_RMD_LIB_MSG) {
        printf("PID param readed fail!\r\n");
      } // return -1;
    }
  };
  // return 0;
}
void rmd_can::read_acc_pid_data(int8_t id) {
  int8_t cmd[8] = {};
  cmd[0] = READ_ACC_ID;
  if (can_send(id, cmd)) {
    if (can_read(id, READ_ACC_ID) == READ_ACC_ID) {
      Accel = (int32_t)(data_buff[4] | data_buff[5] << 8 | data_buff[5] << 16 |
                        data_buff[5] << 24);
      if (DEBUG_RMD_LIB_MSG) {
        printf("ACCEL param updated!\r\n");
      }
    } else {
      if (DEBUG_RMD_LIB_MSG) {
        printf("ACCEL param readed fail!\r\n");
      }
    }
  }
}
void rmd_can::read_encoder(int8_t id) {
  int8_t cmd[8] = {};
  cmd[0] = READ_ENCODER_ID;
  if (can_send(id, cmd)) {
    if (can_read(id, READ_ENCODER_ID) == READ_ENCODER_ID) {
      encoder = (uint16_t)(data_buff[2] | data_buff[3] << 8);
      encoder_raw = (uint16_t)(data_buff[4] | data_buff[5] << 8);
      encoder_raw = (uint16_t)(data_buff[6] | data_buff[7] << 8);
      if (DEBUG_RMD_LIB_MSG) {
        printf("Encoder value updated!\r\n");
      }
    } else {
      if (DEBUG_RMD_LIB_MSG) {
        printf("ACCEL param readed fail!\r\n");
      }
    }
  }
}
void rmd_can::read_global_angle(int8_t id) {
  int8_t cmd[8] = {};
  cmd[0] = READ_GLOBAL_POS_ID;
  if (can_send(id, cmd)) {
    if (can_read(id, READ_GLOBAL_POS_ID) == READ_GLOBAL_POS_ID) {
      pos = (int64_t)(data_buff[0] || data_buff[1] << 8 || data_buff[2] << 16 ||
                      data_buff[3] << 24 ||
                      data_buff[4] << 32 || data_buff[5] << 40 ||
                          data_buff[6] << 48|| data_buff[7] << 56 );
      if (DEBUG_RMD_LIB_MSG) {
        printf("global position updated!\r\n");
      }
    } else {
      if (DEBUG_RMD_LIB_MSG) {
        printf("global position readed fail!\r\n");
      }
    }
  }
}
void rmd_can::read_angle(int8_t id) {
  int8_t cmd[8] = {};
  cmd[0] = READ_LOCAL_POS_ID;
  if (can_send(id, cmd)) {
    if (can_read(id, READ_LOCAL_POS_ID) == READ_LOCAL_POS_ID) {
      angle = (uint16_t)(data_buff[6] | data_buff[7] << 8);
      if (DEBUG_RMD_LIB_MSG) {
        printf("local position updated!\r\n");
      }
    } else {
      if (DEBUG_RMD_LIB_MSG) {
        printf("load position readed fail!\r\n");
      }
    }
  }
}

void rmd_can::motor_disable(int8_t id) {
  cmd[0] = MOTOR_DISABLE_ID;
  if (can_send(id, cmd)) {
    if (can_read(id, MOTOR_DISABLE_ID) == MOTOR_DISABLE_ID) {
      if (DEBUG_RMD_LIB_MSG) {
        printf("Motor power OFF!\r\n");
      }
    } else {
      if (DEBUG_RMD_LIB_MSG) {
        printf("Motor power OFF Fail!\r\n");
      }
    }
  }
}
void rmd_can::motor_enable(int8_t id) {
  cmd[0] = MOTOR_ENABLE_ID;
  if (can_send(id, cmd)) {
    if (can_read(id, MOTOR_ENABLE_ID) == MOTOR_ENABLE_ID) {
      if (DEBUG_RMD_LIB_MSG) {
        printf("Motor power ON!\r\n");
      }
    } else {
      if (DEBUG_RMD_LIB_MSG) {
        printf("Motor power ON Fail!\r\n");
      }
    }
  }
}
void rmd_can::clear_error(int8_t id) {
  cmd[0] = CLEAR_FLAG_ID;
  if (can_send(id, cmd)) {
    if (can_read(id, CLEAR_FLAG_ID) == CLEAR_FLAG_ID) {
      if (DEBUG_RMD_LIB_MSG) {
        printf("Clear Flag!\r\n");
      }
    } else {
      if (DEBUG_RMD_LIB_MSG) {
        printf("Clear Flag Fail!\r\n");
      }
    }
  }
}

/*
void rmd_can::status_update1(int8_t id);
void rmd_can::status_update2(int8_t id);
void rmd_can::status_update3(int8_t id);

int rmd_can::set_zero(int8_t id);
int rmd_can::set_torq(int8_t id,int16_t torq);
int rmd_can::set_velocity(int8_t id,int32_t velocity);
int rmd_can::set_position(int8_t id,int32_t pos);
int rmd_can::set_position_speed(int8_t id,int32_t pos,uint16_t profile_speed);
int rmd_can::set_single_turn_angle(int8_t id,uint16_t angle, uint8_t dir);
int rmd_can::set_single_turn_angle(int8_t id,uint16_t angle, uint8_t dir,
uint16_t profile_speed); int rmd_can::set_pid_RAM(int8_t id,int8_t p_kp, int8_t
p_ki, int8_t v_kp, int8_t v_ki, int8_t i_kp, int8_t i_ki); int
rmd_can::set_pid_ROM(int8_t id,int8_t p_kp, int8_t p_ki, int8_t v_kp, int8_t
v_ki, int8_t i_kp, int8_t i_ki); int rmd_can::set_acc_RAM(int8_t id,int32_t
Accel); int rmd_can::set_encoder_offset(int8_t id,int16_t pos_offset);
*/
