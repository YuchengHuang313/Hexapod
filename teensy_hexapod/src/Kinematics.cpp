#include "Kinematics.h"
#include <math.h>
#include <Arduino.h>

bool verbose = false;
double degPerUnit = 0.24;

/**
 * @brief inverse kinematics function to calculate the angles of the servos
 *
 * @param outputs array to store the calculated angles
 * @param x destination x coordinate
 * @param y destination y coordinate
 * @param z destination z coordinate
 * @param a1 distance from hip servo to knee servo
 * @param a2 distance from knee servo to ankle servo
 * @param a3 distance from ankle servo to the foot
 * @return true if the inverse kinematics is successful
 * @return false if the destination is out of reach
 */

bool inverse_kinematics(double *outputs, double x, double y, double z, double a1, double a2, double a3)
{
    double target_dis = sqrt(sq(x) + sq(y) + sq(z));
    double max_leg_dis = a1 + a2 + a3;
    if (max_leg_dis < target_dis)
    {
        if (verbose)
        {
            Serial.println("Kinematics.cpp -> inverse_kinematics: Out of reach");
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
    double phi1 = acos(constrain((sq(a3) - sq(a2) - sq(r3)) / (-2 * a2 * r3), -1, 1)); // limit the range of acos to prevent error
    double theta2 = phi1 + phi2;
    double phi3 = acos(constrain((sq(r3) - sq(a2) - sq(a3)) / (-2 * a2 * a3), -1, 1)); // limit the range of acos to prevent error
    double theta3 = -(PI - phi3);

    theta1 = rad2Deg(theta1);
    theta2 = rad2Deg(theta2);
    theta3 = rad2Deg(theta3);

    // Verbose output if enabled
    if (verbose)
    {
        Serial.printf("hip_angle_deg:%10.3f\n", theta1);
        Serial.printf("r1 value:    %10.3f\n", r1);
        Serial.printf("r2 value:    %10.3f\n", r2);
        Serial.printf("phi2 value:  %10.3f\n", phi2);
        Serial.printf("r3 value:    %10.3f\n", r3);
        Serial.printf("phi1 value:  %10.3f\n", phi1);
        Serial.printf("knee_angle_deg:%10.3f\n", theta2);
        Serial.printf("phi3 value:  %10.3f\n", phi3);
        Serial.printf("ankle_angle_deg:%10.3f\n", theta3);
    }

    outputs[0] = theta1;
    outputs[1] = theta2;
    outputs[2] = theta3; // ankle motor is flipped
    return true;
}
/**
 * @brief forward kinematics function to calculate the x, y, z coordinates of the leg
 *
 * @param outputs array to store the x, y, z coordinates
 * @param range_limits range limits of the servos
 * @param theta1 hip servo angle
 * @param theta2 knee servo angle
 * @param theta3 ankle servo angle
 * @param a1 distance from hip to knee
 * @param a2 distance from knee to ankle
 * @param a3 distance from ankle to foot
 * @return true if the forward kinematics is successful
 * @return false if the input angles are out of reach
 */
bool forward_kinematics(double *outputs, double *range_limits, double hip_angle_deg, double knee_angle_deg, double ankle_angle_deg, double a1, double a2, double a3)
{
    // Convert thetas from degrees to radians
    double hip_angle_rad = deg2Rad(hip_angle_deg);
    double knee_angle_rad = deg2Rad(knee_angle_deg);
    double ankle_angle_rad = deg2Rad(ankle_angle_deg);
    // Check if the input angles are within the defined limits
    if (hip_angle_deg < range_limits[0] || hip_angle_deg > range_limits[1] ||
        knee_angle_deg < range_limits[2] || knee_angle_deg > range_limits[3] ||
        ankle_angle_deg < range_limits[4] || ankle_angle_deg > range_limits[5])
    {
        Serial.println("Kinematics.cpp -> forward_kinematics: Out of reach");
        return false;
    }

    // Calculate r1, r2, r3 based on theta1, theta2, theta3
    double r1 = a1 + a2 * cos(knee_angle_rad) + a3 * cos(knee_angle_rad + ankle_angle_rad);
    double r2 = a2 * sin(knee_angle_rad) + a3 * sin(knee_angle_rad + ankle_angle_rad);

    // Calculate x, y, z coordinates
    double x = r1 * cos(hip_angle_rad);
    double y = r1 * sin(hip_angle_rad);
    double z = r2;

    // Print forward kinematics results
    if (verbose)
    {
        Serial.println("\nForward kinematics results:");
        Serial.print("input hip angle: ");
        Serial.println(hip_angle_deg);
        Serial.print("input keen angle: ");
        Serial.println(knee_angle_deg);
        Serial.print("input ankle angle: ");
        Serial.println(ankle_angle_deg);
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

double deg2unit(double deg)
{
    return (deg / degPerUnit + 500); // Ensure degPerUnit value is directly included here
}

double rad2Deg(double rad)
{
    return rad * (180.0 / PI);
}

double deg2Rad(double deg)
{
    return deg * (PI / 180.0);
}