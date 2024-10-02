#ifndef SERVOUTILS_H
#define SERVOUTILS_H

void servo_check(int* available_servo_ids);
float deg2unit(float deg);
bool limitation_check(float* inputs, float* range_limits);

#endif
