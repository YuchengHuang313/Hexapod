#include "Kinematics.h"
#include <math.h>
#include <Arduino.h>

extern bool verbose = false;
extern double degPerUnit = 0.24;

int inverse_kinematics(double* arr, double x, double y, double z, double a1, double a2, double a3) {
    double target_dis = sqrt(sq(x) + sq(y) + sq(z));
    double max_leg_dis = a1 + a2 + a3;
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

    arr[0] = theta1;
    arr[1] = theta2;
    arr[2] = -theta3; // ankle motor is flipped
    return 1;
}

double deg2unit(double deg) {
    return (deg / degPerUnit + 500);  // Ensure degPerUnit value is directly included here
}

double rad2Deg(double rad) {
    return rad * (180.0 / PI);
}
