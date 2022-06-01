#include "vesc_can_ard.h"

//CAN
#define vesc1_id  81
#define vesc2_id  82
#define vesc3_id  83
#define vesc4_id  84

void setup() {
  Serial.begin(115200);
  vesc_init();
  reset_param();
}

void loop() {
  set_rpm(vesc1_id, 1200);
  set_rpm(vesc2_id, 1200);
  set_rpm(vesc3_id, 1200);
  set_rpm(vesc4_id, 1200);
}
