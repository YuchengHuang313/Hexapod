#include "Kinematics.h"
#include <math.h>
#include <Arduino.h>

bool verbose = false;
double degPerUnit = 0.24;

bool inverse_kinematics(double* outputs, double x, double y, double z, double a1, double a2, double a3) {
  double target_dis = sqrt(sq(x) + sq(y) + sq(z));
  double max_leg_dis = a1 + a2 + a3;
  if (max_leg_dis >= target_dis) {
    if (verbose) {
      Serial.println("reachable");
    }
  } else {
    if (verbose) {
      Serial.printf("Position (%f, %f, %f) is ", x, y, z);
      Serial.println("unreachable");
    }
    return false;
  }

  // Top view calculation, finding theta1
  double theta1 = atan2(y, x);
  double r1 = sqrt(sq(x) + sq(y)) - a1;

  // Side view calculation, finding theta2 and theta3
  double r2 = z;
  double phi2 = atan2(r2, r1);
  double r3 = sqrt(sq(r1) + sq(r2));
  double phi1 = acos(constrain((sq(a3) - sq(a2) - sq(r3)) / (-2 * a2 * r3), -1, 1));  // limit the range of acos to prevent error
  double theta2 = phi1 + phi2;
  double phi3 = acos(constrain((sq(r3) - sq(a2) - sq(a3)) / (-2 * a2 * a3), -1, 1));  // limit the range of acos to prevent error
  double theta3 = -(PI - phi3);

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

  outputs[0] = theta1;
  outputs[1] = theta2;
  outputs[2] = theta3;  // ankle motor is flipped
  return true;
}

bool forward_kinematics(double* outputs, double* range_limits, double theta1, double theta2, double theta3, double a1, double a2, double a3) {
  // Convert thetas from degrees to radians
  double theta1_rad = deg2Rad(theta1);
  double theta2_rad = deg2Rad(theta2);
  double theta3_rad = deg2Rad(theta3);
  // Check if the input angles are within the defined limits
  if (theta1 < range_limits[0] || theta1 > range_limits[1] || theta2 < range_limits[2] || theta2 > range_limits[3] || theta3 < range_limits[4] || theta3 > range_limits[5]) {
    Serial.println("Out of reach");
    return false;
  }

  // Calculate r1, r2, r3 based on theta1, theta2, theta3
  double r1 = a1 + a2 * cos(theta2_rad) + a3 * cos(theta2_rad + theta3_rad);
  double r2 = a2 * sin(theta2_rad) + a3 * sin(theta2_rad + theta3_rad);
  double r3 = r1 * cos(theta1_rad);

  // Calculate x, y, z coordinates
  double x = r3 * cos(theta1_rad);
  double y = r3 * sin(theta1_rad);
  double z = r2;

  // Print forward kinematics results
  if (verbose) {
    Serial.println("\nForward kinematics results:");
    Serial.print("input x: ");
    Serial.println(outputs[0]);
    Serial.print("input y: ");
    Serial.println(outputs[1]);
    Serial.print("input z: ");
    Serial.println(outputs[2]);
    Serial.print("x coordinate: ");
    Serial.println(x);
    Serial.print("y coordinate: ");
    Serial.println(y);
    Serial.print("z coordinate: ");
    Serial.println(z);
  }


  // Store the results in the output array
  outputs[0] = x;
  outputs[1] = y;
  outputs[2] = z;

  return true;
}

int deg2unit(double deg) {
  return (int)(deg / degPerUnit + 500);  // Ensure degPerUnit value is directly included here
}

double unit2deg(int unit) {
  return (unit - 500) * degPerUnit;
}

double rad2Deg(double rad) {
  return rad * (180.0 / PI);
}

double deg2Rad(double deg) {
  return deg * (PI / 180.0);
}