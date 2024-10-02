#include "Kinematics.h"
#include <math.h>
#include <Arduino.h>

extern bool verbose;
extern const float a1;
extern const float a2;
extern const float a3;

int inverse_kinematics(float* arr, float x, float y, float z) {
  float target_dis = sqrt(sq(x) + sq(y) + sq(z));
  float max_leg_dis = a1 + a2 + a3;
  if (max_leg_dis >= target_dis) {
    if (verbose) {
      Serial.println("reachable");
    }
  } else {
    if (verbose) {
      Serial.println("unreachable");
    }
    return -1;
  }

  // Top view calculation, finding theta1
  float theta1 = atan2(y, x);
  float r1 = sqrt(sq(x) + sq(y)) - a1;

  // Side view calculation, finding theta2 and theta3
  float r2 = z;
  float phi2 = atan2(r2, r1);
  float r3 = sqrt(sq(r1) + sq(r2));
  float phi1 = acos(constrain((sq(a3) - sq(a2) - sq(r3)) / (-2 * a2 * r3), -1, 1));  // limit the range of acos to prevent error
  float theta2 = phi1 + phi2;
  float phi3 = acos(constrain((sq(r3) - sq(a2) - sq(a3)) / (-2 * a2 * a3), -1, 1));  // limit the range of acos to prevent error
  float theta3 = -(PI - phi3);

  theta1 = rad2Deg(theta1);
  theta2 = rad2Deg(theta2);
  theta3 = rad2Deg(theta3);

  // Verbose output if enabled
  if (verbose) {
    Serial.printf("theta1 value:%10.3f\n", theta1);
    Serial.printf("r1 value:    %10.3f\n", r1);
    Serial.printf("r2 value:    %10.3f\n", r2);
    Serial.printf("phi2 value:  %10.3f\n", phi2);
    Serial.printf("r3 value:    %10.3f\n", r3);
    Serial.printf("phi1 value:  %10.3f\n", phi1);
    Serial.printf("theta2 value:%10.3f\n", theta2);
    Serial.printf("phi3 value:  %10.3f\n", phi3);
    Serial.printf("theta3 value:%10.3f\n", theta3);
  }

  arr[0] = theta1;
  arr[1] = theta2;
  arr[2] = -theta3; // ankle motor is flip
  return 1;
}

float rad2Deg(float rad) {
  return rad * (180.0 / PI);
}
