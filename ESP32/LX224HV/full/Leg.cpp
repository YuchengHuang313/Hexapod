#include "esp32-hal.h"
#include "HardwareSerial.h"
#include <math.h>
#include "Leg.h"
#include "Kinematics.h"

// Static member initialization
bool Leg::verbose = true;
// move range limitation for hip, knee, and ankle servos in degrees
// since the hexapod enforces elbow up gait, the ankle servo can never be lower than 500 unit position, which is 0 degree
// due to float point error, upper limit of the ankle is set to a very small positive number (0.1) instead of 0
double Leg::range_limits[6] = { -30.1, 30.1, -90.1, 90.1, -120.1, 0.1 };
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

  if (verbose) {
    Serial.printf("-------------------- Initializing leg %i --------------------\n", this->legId);
  }
}

// Method to move the leg to a given position
bool Leg::leg_to_position(double x, double y, double z) {
  if (verbose) {
    Serial.printf("-------------------- Moving leg %i --------------------\n", this->legId);
  }
  if (inverse_kinematics(this->thetas, x, y, z, Leg::A1, Leg::A2, Leg::A3) == 1) {
    if (leg_check_range_limits(this->thetas)) {
      Leg::busServo.LobotSerialServoMove(this->hipId, deg2unit(this->thetas[0]), 1000);
      Leg::busServo.LobotSerialServoMove(this->kneeId, deg2unit(this->thetas[1]), 1000);
      Leg::busServo.LobotSerialServoMove(this->ankleId, deg2unit(this->thetas[2]), 1000);
      leg_update_position();
      return true;
    }
  }
  return false;
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
  for (int i = leg_get_id(); i <= leg_get_id() * Leg::TOTAL_LEG_SERVOS; i++) {
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

double* Leg::leg_get_cur_local_pos() {
  if (verbose) {
    Serial.printf("-------------------- Leg %i calculated servos positions --------------------\n", leg_get_id());
    Serial.printf("Hip servo: %f\n", this->servo_position[0]);
    Serial.printf("Knee servo: %f\n", this->servo_position[1]);
    Serial.printf("Ankle servo: %f\n", this->servo_position[2]);
  }
  return this->servo_position;
}

void Leg::leg_update_position() {
  if (verbose) {
    Serial.printf("-------------------- Update leg %i servos positions --------------------\n", leg_get_id());
  }
  // calculate the servo positions from stored servo angles
  for (int i = 0; i < Leg::TOTAL_LEG_SERVOS; i++) {
    this->servo_position[i] = deg2unit(this->thetas[i]);
  }
}

void Leg::leg_check_angle_error() {
  if (verbose) {
    Serial.printf("-------------------- Checking leg %i angle error --------------------\n", this->legId);
  }
  delay(3000); // make sure all servos have stopped moving
  double actual_pos[3] = { -9999, -9999, -9999 };
  int acceptable_error_range_unit = 10;

  for (int i = 0; i < Leg::TOTAL_LEG_SERVOS; i++) {
    actual_pos[i] = Leg::busServo.LobotSerialServoReadPosition(i + 1);
    if (abs(deg2unit(this->thetas[i]) - actual_pos[i]) > acceptable_error_range_unit) {
      Serial.println("SERVO ANGLES DISAGREE TOO MUCH, ENDPOINT MIGHT BE INACCURATE!");
    }
  }

  if (verbose) {
    Serial.println("Calculated positions:");
    Serial.printf("Hip servo:\t%6.3f\n", deg2unit(this->thetas[0]));
    Serial.printf("Knee servo:\t%6.3f\n", deg2unit(this->thetas[1]));
    Serial.printf("Ankle servo:\t%6.3f\n", deg2unit(this->thetas[2]));
    Serial.println("Actual positions: ");
    Serial.printf("Hip servo:\t%6.3f\n", actual_pos[0]);
    Serial.printf("Knee servo:\t%6.3f\n", actual_pos[1]);
    Serial.printf("Ankle servo:\t%6.3f\n", actual_pos[2]);
  }
}