#ifndef LEG_H
#define LEG_H

#include "LobotSerialServoControl.h"
#include <math.h>

class Leg {
private:
  // instance fields to be used
  int legId;
  int hipId;
  int kneeId;
  int ankleId;
  double thetas[3];
  double servo_position[3];

  // static fields for common features among all legs
  static constexpr double A1 = 37.0;
  static constexpr double A2 = 63.54;
  static constexpr double A3 = 200.0;
  static constexpr int TOTAL_LEG_SERVOS = 3;
  static constexpr int FOOT_DEV_ANGLE = 15;
  static constexpr double DEGREE_PER_UNIT = 0.24;
  static constexpr int FOOT_UNIT_OFFSET = ceil(FOOT_DEV_ANGLE / DEGREE_PER_UNIT); // LX224hv has 0.24 degree per unit
  static bool verbose;
  static double range_limits[6];

  // all servos will be connected to hardwareSerial 1 port with baud 115200, TX 17, and RX 16
  static HardwareSerial hardwareSerial;
  static LobotSerialServoControl busServo;
  static constexpr int BAUDRATE = 115200;
  static constexpr int SERVO_SERIAL_TX = 17;
  static constexpr int SERVO_SERIAL_RX = 16;

public:
  Leg(int leg_id, int hip_id, int knee_id, int ankle_id);
  bool leg_to_position(double x, double y, double z);
  bool leg_check_status();
  void leg_toggle_verbose();
  void leg_unload_all();
  void leg_load_all();
  double* leg_get_range_limits();
  void leg_set_range_limits(double* range);
  int leg_get_id();
  int leg_get_hip_id();
  int leg_get_knee_id();
  int leg_get_ankle_id();
  double* leg_get_cur_local_pos();
  void leg_check_angle_error();
protected:
  bool leg_check_range_limits(double* input_angles);
  void leg_update_position();
  
};

#endif  // LEG_H