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

  double x = a1 + a3 ;
  double y = 0;
  double z = a2;

  Leg leg1(1, 1, 2, 3);
  leg1.leg_to_position(x, 150, z);
  delay(1000);
  for (int i = 1; i <= 100; i++){
    leg1.leg_to_position(x, 150 - 3*i, z);
  }

  
  delay(1000);
  //leg1.leg_step_to_position(x, -150, z);

  // leg1.leg_to_position(x, -150, z);
  // leg1.leg_to_position(x, 100, z);
  // delay(1000);
  // leg1.leg_to_position(x, 0, z);
  // delay(1000);
  // leg1.leg_to_position(x, -100, z);
  // double* positions = leg1.leg_get_cur_local_units();



  delay(3000);
  leg1.leg_unload_all();
  Serial.println("Done");
}

void loop() {}
