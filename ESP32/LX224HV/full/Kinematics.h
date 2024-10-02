#ifndef KINEMATICS_H
#define KINEMATICS_H

bool inverse_kinematics(double* outputs, double x, double y, double z, double a1, double a2, double a3);
bool forward_kinematics(double* outputs, double* range_limits, double x, double y, double z, double a1, double a2, double a3);
double deg2unit(double deg);
double rad2Deg(double rad);
double deg2Rad(double deg);

#endif // KINEMATICS_H
