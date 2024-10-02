#include "esp32-hal.h"
#include "HardwareSerial.h"
#include <math.h>
#include "Leg.h"
#include "Kinematics.h"

// Static member initialization
bool Leg::verbose = false;

// move range limitation for hip, knee, and ankle servos in degrees
// since the hexapod enforces elbow up gait, the ankle servo can never be higher than 500 unit position, which are 0+ degrees
// due to float point error, upper limit of the ankle is set to a very small positive number (0.1) instead of 0
// lower hip, upper hip, lower knee, upper knee, lower ankle, upper ankle
double Leg::range_limits[6] = { -60.1, 60.1, -90.1, 90.1, -120.1, 0.1 };

HardwareSerial Leg::hardwareSerial(1);
LobotSerialServoControl Leg::busServo(Leg::hardwareSerial);

// Constructor to initialize servo IDs and set up the bus servo
Leg::Leg(int leg_id, int hip_id, int knee_id, int ankle_id) {
  this->legId = leg_id;
  this->hipId = hip_id;
  this->kneeId = knee_id;
  this->ankleId = ankle_id;

  Leg::hardwareSerial.begin(Leg::BAUDRATE, SERIAL_8N1, Leg::SERVO_SERIAL_TX, Leg::SERVO_SERIAL_RX);
  Leg::busServo.OnInit();
  Leg::busServo.lobotSerialServoOffsetWrite(this->ankleId, Leg::FOOT_UNIT_OFFSET);

  leg_update_all();

  Leg::leg_update_positions();

  if (verbose) {
    Serial.printf("-------------------- Initializing leg %i --------------------\n", this->legId);
  }
}

// Method to move the leg to a given position
bool Leg::leg_to_position(double targetX, double targetY, double targetz) {
  if (verbose) {
    Serial.printf("-------------------- Moving leg %i --------------------\n", this->legId);
  }
  if (inverse_kinematics(this->servo_angles, targetX, targetY, targetz, Leg::A1, Leg::A2, Leg::A3)) {
    if (leg_check_range_limits(this->servo_angles)) {
      Leg::busServo.LobotSerialServoMove(this->hipId, deg2unit(this->servo_angles[0]), 1);
      Leg::busServo.LobotSerialServoMove(this->kneeId, deg2unit(this->servo_angles[1]), 1);
      Leg::busServo.LobotSerialServoMove(this->ankleId, deg2unit(this->servo_angles[2]), 1);
      while (abs(Leg::busServo.LobotSerialServoReadPosition(this->hipId) - deg2unit(this->servo_angles[0])) > 5
             && abs(Leg::busServo.LobotSerialServoReadPosition(this->kneeId) - deg2unit(this->servo_angles[1])) > 5
             && abs(Leg::busServo.LobotSerialServoReadPosition(this->ankleId) - deg2unit(this->servo_angles[2])) > 5){
          continue;
        }
      // leg_update_all();
      return true;
    }
  }
  return false;
}

bool Leg::leg_step_to_position(double targetX, double targetY, double targetZ) {
  if (verbose) {
    Serial.printf("-------------------- Stepping leg %i --------------------\n", this->legId);
  }
  leg_update_all();
  int steps = 50;                                  // Sample 100 points on the straight line between starting and ending point
  double starting_pos_x = this->leg_positions[0];  // get starting hip servo unit
  double starting_pos_y = this->leg_positions[1];  // get starting knee servo unit
  double starting_pos_z = this->leg_positions[2];  // get starting ankle servo unit

  // cal for the amount of movement for each steps
  double delta_x = (targetX - starting_pos_x) / steps;
  double delta_y = (targetY - starting_pos_y) / steps;
  double delta_z = (targetZ - starting_pos_z) / steps;

  double angles[3] = { 0, 0, 0 };
  for (int i = 1; i <= steps; i++) {
    if (inverse_kinematics(angles, starting_pos_x + delta_x * i, starting_pos_y + delta_y * i, starting_pos_z + delta_z * i, Leg::A1, Leg::A2, Leg::A3)) {
      Serial.println("herer");
      // Leg::busServo.LobotSerialServoMove(this->hipId, deg2unit(angles[0]), 1);
      // Leg::busServo.LobotSerialServoMove(this->kneeId, deg2unit(angles[1]), 1);
      // Leg::busServo.LobotSerialServoMove(this->ankleId, deg2unit(angles[2]), 1);
    }
    // leg_to_position(starting_pos_x + delta_x * i, starting_pos_y + delta_y * i, starting_pos_z + delta_z * i);

    // for every 10 steps taken, check for deviation of each servo, and wait for the last move to complete
    while (i % (steps / 1) == 0
           && abs(Leg::busServo.LobotSerialServoReadPosition(this->hipId) - starting_pos_x + delta_x * i) > 25
           && abs((Leg::busServo.LobotSerialServoReadPosition(this->kneeId) - starting_pos_y + delta_y * i) > 25)
           && abs((Leg::busServo.LobotSerialServoReadPosition(this->ankleId) - starting_pos_z + delta_z * i) > 25)) {
      continue;
    }
  }

  return true;
}

// Method to check the status of the leg
bool Leg::leg_check_status() {
  if (verbose) {
    Serial.printf("-------------------- Checking leg %i status --------------------\n", leg_get_id());
  }
  bool flag = true;
  for (int i = 1; i <= Leg::TOTAL_LEG_SERVOS; i++) {
    if (Leg::busServo.LobotSerialServoReadPosition(i) == -1) {
      if (verbose) {
        Serial.printf("Servo with ID of %i NOT found on hexapod", i);
        Serial.println();
      }
      flag = false;
    } else {
      if (verbose) {
        Serial.printf("Servo with ID of %i found on hexapod", i);
        Serial.println();
      }
    }
  }
  if (flag) {
    if (verbose) {
      Serial.printf("Leg %i has hip servo %i, knee servo %i, ankle servo %i detected", this->legId, this->hipId, this->kneeId, this->ankleId);
      Serial.println();
    }
    return true;
  }
  return false;
}

// Method to toggle verbosity
void Leg::leg_toggle_verbose() {
  if (verbose) {
    Serial.printf("-------------------- Toggling leg %i verbose --------------------\n", leg_get_id());
  }
  Leg::verbose = !Leg::verbose;
}

void Leg::leg_unload_all() {
  if (verbose) {
    Serial.printf("-------------------- Unloading all leg %i servos --------------------\n", leg_get_id());
  }
  for (int i = 1; i <= Leg::TOTAL_LEG_SERVOS; i++) {
    Leg::busServo.LobotSerialServoUnload(i);
  }
}

void Leg::leg_load_all() {
  if (verbose) {
    Serial.printf("-------------------- Loading all leg %i servos --------------------\n", leg_get_id());
  }
  for (int i = 1; i <= Leg::TOTAL_LEG_SERVOS; i++) {
    Leg::busServo.LobotSerialServoLoad(i);
  }
}

double* Leg::leg_get_range_limits() {
  return Leg::range_limits;
}

void Leg::leg_set_range_limits(double* range) {
  for (int i = 0; i < Leg::TOTAL_LEG_SERVOS * 2; i++) {
    Leg::range_limits[i] = range[i];
  }
}

bool Leg::leg_check_range_limits(double* input_angles) {
  if (verbose) {
    Serial.printf("-------------------- Checking leg %i angle limits --------------------\n", this->legId);
  }
  bool withinLimits = true;

  // Check Hip Servo Limits
  if (input_angles[0] < Leg::range_limits[0]) {
    if (Leg::verbose) {
      Serial.printf("Hip servo angle %f is below minimum limit of %f\n", input_angles[0], Leg::range_limits[0]);
    }
    withinLimits = false;
  } else if (Leg::verbose) {
    Serial.printf("Hip servo angle %f is safe under maximum limit of %f\n", input_angles[0], Leg::range_limits[1]);
  }

  if (input_angles[0] > Leg::range_limits[1]) {
    if (Leg::verbose) {
      Serial.printf("Hip servo angle %f is above maximum limit of %f\n", input_angles[0], Leg::range_limits[1]);
    }
    withinLimits = false;
  }

  // Check Knee Servo Limits
  if (input_angles[1] < Leg::range_limits[2]) {
    if (Leg::verbose) {
      Serial.printf("Knee servo angle %f is below minimum limit of %f\n", input_angles[1], Leg::range_limits[2]);
    }
    withinLimits = false;
  } else if (Leg::verbose) {
    Serial.printf("Knee servo angle %f is safe under maximum limit of %f\n", input_angles[1], Leg::range_limits[3]);
  }

  if (input_angles[1] > Leg::range_limits[3]) {
    if (Leg::verbose) {
      Serial.printf("Knee servo angle %f is above maximum limit of %f\n", input_angles[1], Leg::range_limits[3]);
    }
    withinLimits = false;
  }

  // Check Ankle Servo Limits
  if (input_angles[2] < Leg::range_limits[4]) {
    if (Leg::verbose) {
      Serial.printf("Ankle servo angle %f is below minimum limit of %f\n", input_angles[2], Leg::range_limits[4]);
    }
    withinLimits = false;
  } else if (Leg::verbose) {
    Serial.printf("Ankle servo angle %f is safe under maximum limit of %f\n", input_angles[2], Leg::range_limits[5]);
  }

  if (input_angles[2] > Leg::range_limits[5]) {
    if (Leg::verbose) {
      Serial.printf("Ankle servo angle %f is above maximum limit of %f\n", input_angles[2], Leg::range_limits[5]);
    }
    withinLimits = false;
  }

  return withinLimits;
}

int Leg::leg_get_id() {
  return this->legId;
}

int Leg::leg_get_hip_id() {
  return this->hipId;
}

int Leg::leg_get_knee_id() {
  return this->kneeId;
}

int Leg::leg_get_ankle_id() {
  return this->ankleId;
}

double* Leg::leg_get_cur_local_units() {
  if (verbose) {
    Serial.printf("-------------------- Leg %i calculated servos positions --------------------\n", leg_get_id());
    Serial.printf("Hip servo: %f\n", this->servo_units[0]);
    Serial.printf("Knee servo: %f\n", this->servo_units[1]);
    Serial.printf("Ankle servo: %f\n", this->servo_units[2]);
  }
  return this->servo_units;
}


// Protected methods
void Leg::leg_update_units() {
  if (verbose) {
    Serial.printf("-------------------- Update leg %i servos units --------------------\n", leg_get_id());
  }
  // calculate the servo positions from stored servo angles
  for (int i = 1; i <= Leg::TOTAL_LEG_SERVOS; i++) {  // servos are index 1 based
    this->servo_units[i - 1] = Leg::busServo.LobotSerialServoReadPosition(i);
    if (verbose) {
      Serial.printf("leg %i's servo angle %i is written with %f\n", leg_get_id(), i, this->servo_units[i - 1]);
    }
  }
}

void Leg::leg_update_angles() {
  if (verbose) {
    Serial.printf("-------------------- Update leg %i servos angles --------------------\n", leg_get_id());
  }
  for (int i = 0; i < Leg::TOTAL_LEG_SERVOS; i++) {
    this->servo_angles[i] = unit2deg(this->servo_units[i]);
    if (verbose) {
      Serial.printf("leg %i's servo angle %i is written with %f\n", leg_get_id(), i, this->servo_angles[i]);
    }
  }
}

void Leg::leg_update_positions() {
  if (verbose) {
    Serial.printf("-------------------- Update leg %i servos positions --------------------\n", leg_get_id());
  }
  forward_kinematics(this->leg_positions, Leg::range_limits, this->servo_angles[0], this->servo_angles[1], this->servo_angles[2], Leg::A1, Leg::A2, Leg::A3);
}

void Leg::leg_update_all() {
  leg_update_units();
  leg_update_angles();
  leg_update_positions();
}

void Leg::leg_check_angle_error() {
  if (verbose) {
    Serial.printf("-------------------- Checking leg %i angle error --------------------\n", this->legId);
  }
  delay(3000);  // make sure all servos have stopped moving
  double actual_pos[3] = { -9999, -9999, -9999 };
  int acceptable_error_range_unit = 10;

  for (int i = 0; i < Leg::TOTAL_LEG_SERVOS; i++) {
    actual_pos[i] = Leg::busServo.LobotSerialServoReadPosition(i + 1);
    if (abs(deg2unit(this->servo_angles[i]) - actual_pos[i]) > acceptable_error_range_unit) {
      Serial.println("SERVO ANGLES DISAGREE TOO MUCH, ENDPOINT MIGHT BE INACCURATE!");
    }
  }

  if (verbose) {
    Serial.println("Calculated positions:");
    Serial.printf("Hip servo:\t%6.3f\n", deg2unit(this->servo_angles[0]));
    Serial.printf("Knee servo:\t%6.3f\n", deg2unit(this->servo_angles[1]));
    Serial.printf("Ankle servo:\t%6.3f\n", deg2unit(this->servo_angles[2]));
    Serial.println("Actual positions: ");
    Serial.printf("Hip servo:\t%6.3f\n", actual_pos[0]);
    Serial.printf("Knee servo:\t%6.3f\n", actual_pos[1]);
    Serial.printf("Ankle servo:\t%6.3f\n", actual_pos[2]);
  }
}