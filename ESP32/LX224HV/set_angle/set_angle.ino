#include "LobotSerialServoControl.h"


#define SERVO_SERIAL_TX 17
#define SERVO_SERIAL_RX 16
#define total_num_servos 18

const float degPerUnit = 0.24;
const int legDevAngle = 15;
const float a1 = 37;
const float a2 = 63.54;
const float a3 = 150;

bool verbose = true;

HardwareSerial HardwareSerial(1);
LobotSerialServoControl BusServo(HardwareSerial);

void setup() {
  Serial.begin(115200);
  Serial.println("start...");
  HardwareSerial.begin(115200, SERIAL_8N1, SERVO_SERIAL_TX, SERVO_SERIAL_RX);  // baud rate, protocol slave tx pin (master rx pin),
                                                                               // slave serial rx pin (master tx pin).
  BusServo.OnInit();
  delay(500);

  int available_servo_ids[total_num_servos];  // Declare as an array of integers
  servo_check(available_servo_ids);           // Pass it to the function

  float* array = (float*)malloc(3 * sizeof(float));
  Serial.println(inverse_kinematics(array, a1+a2, 0, -a3));

  Serial.println(array[0]);
  Serial.println(array[1]);
  Serial.println(array[2]);

  Serial.println(deg2unit(array[0]));
  Serial.println(deg2unit(array[1]));
  Serial.println(deg2unit(array[2]));

  if (array[0] >= -30 && array[0] <= 30 && array[1] >= -90 && array[1] <= 90 && array[2] >= -105 && array[2] <= 105) {
    Serial.println("Moving");
    BusServo.LobotSerialServoMove(1, deg2unit(array[0]), 1000);
    BusServo.LobotSerialServoMove(2, deg2unit(array[1]), 1000);
    BusServo.LobotSerialServoMove(3, deg2unit(array[2]), 1000);
  }
}

void loop() {}

void servo_check(int* available_servo_ids) {
  for (int i = 1; i <= total_num_servos; i++) {  // Check what servos are connected to the hexapod
    if (BusServo.LobotSerialServoReadPosition(i) == -1) {
      Serial.printf("Servo with ID of %i NOT found on hexapod", i);
      Serial.println();
      available_servo_ids[i - 1] = i;
    } else {
      Serial.printf("Servo with ID of %i found on hexapod", i);
      Serial.println();
      available_servo_ids[i - 1] = -1;
    }
  }
}

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

// LX224hv takes a value from 0 to 1000, cooresponding to 0 to 240 degree
// but we defiend when the servo is at its default position, it should be at the middle, the 500 unit
// so a negative angle means the servo moves under 500 unit, a positive angle means the servo move above 500 unit
// e.g. -45 degrees mean -45 / 0.24 + 500 = 312.5 unit
//      45 degrees mean 45 / 0.24 + 500 = 687.5 unit
float deg2unit(float deg) {
  return (deg / degPerUnit + 500);
}

float rad2Deg(float rad) {
  return rad * (180.0 / PI);
}