#include "LobotSerialServoControl.h"
#include "Hexapod.h"
#include "Kinematics.h"

const int total_num_servos = 18;
const float a1 = 37;
const float a2 = 63.54;
const float a3 = 150;

// move range limitation for hip, knee, and ankle servos in degrees
// since the hexapod enforces elbow up gait, the ankle servo can never be lower than 500 unit position, which is 0 degree
// due to float point error, lower limit of the ankle is set to a very small negative number (-0.1) instead of 0

void setup() {
  Serial.begin(115200);
  Serial.println("start...");

  Leg leg5(1, 1, 2, 3);
  // leg5.leg_toggle_verbose();
  leg5.leg_check_status();
  leg5.leg_to_position(a1+a3, 0, a2);
  double* positions = leg5.leg_get_cur_local_pos();
  leg5.leg_check_angle_error();
  delay(3000);
  leg5.leg_unload_all();
  Serial.println("Done");
}

void loop() {}
