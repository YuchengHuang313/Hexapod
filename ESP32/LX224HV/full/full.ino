#include "LobotSerialServoControl.h"
#include "Hexapod.h"
#include "Kinematics.h"

const int total_num_servos = 18;
const float a1 = 37;
const float a2 = 63.54;
const float a3 = 200;

// move range limitation for hip, knee, and ankle servos in degrees
// since the hexapod enforces elbow up gait, the ankle servo can never be lower than 500 unit position, which is 0 degree
// due to float point error, lower limit of the ankle is set to a very small negative number (-0.1) instead of 0

void setup() {
  Serial.begin(115200);
  Serial.println("start...");

  double x = a1 + a3;
  double y = 0;
  double z = a2;
  double output_positions[3] = { 0, 0, 0 };
  double range_limits[6] = { -30.1, 30.1, -90.1, 90.1, -120.1, 0.1 };

  if (forward_kinematics(output_positions, range_limits, 0, 90, -90, a1, a2, a3)) {
    Serial.println("forward_kinematics completed");
  }

  Serial.println(output_positions[0]);
  Serial.println(output_positions[1]);
  Serial.println(output_positions[2]);

  Leg leg1(1, 1, 2, 3);
  leg1.leg_to_position(output_positions[0], output_positions[1], output_positions[2]);
  leg1.leg_get_cur_local_pos();

  Leg leg2(2, 4, 5, 6);
  leg2.leg_to_position(output_positions[0], output_positions[1], output_positions[2]);
  leg2.leg_get_cur_local_pos();

  delay(3000);
  leg1.leg_unload_all();
  leg2.leg_unload_all();
  Serial.println("Done");
}

void loop() {}
