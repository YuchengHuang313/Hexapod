#include "ServoUtils.h"
#include "LobotSerialServoControl.h"

// Ensure the global variables and objects are declared before use
extern LobotSerialServoControl BusServo;
extern const int total_num_servos;

void servo_check(int* available_servo_ids) {
  for (int i = 1; i <= total_num_servos; i++) {  // Check what servos are connected to the hexapod
    if (BusServo.LobotSerialServoReadPosition(i) == -1) {
      Serial.printf("Servo with ID of %i NOT found on hexapod", i);
      Serial.println();
      available_servo_ids[i - 1] = -1;
    } else {
      Serial.printf("Servo with ID of %i found on hexapod", i);
      Serial.println();
      available_servo_ids[i - 1] = i;
    }
  }
}

float deg2unit(float deg) {
  return (deg / 0.24 + 500);  // Ensure degPerUnit value is directly included here
}

bool limitation_check(float* inputs, const float* range_limits) {
  return (inputs[0] >= range_limits[0]) && (inputs[0] <= range_limits[1]) && (inputs[1] >= range_limits[2]) && (inputs[1] <= range_limits[3]) && (inputs[2] >= range_limits[4]) && (inputs[1] <= range_limits[5]);
}