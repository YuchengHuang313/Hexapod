#include "LobotSerialServoControl.h"
#include "ServoUtils.h"
#include "Kinematics.h"

#define SERVO_SERIAL_TX 17
#define SERVO_SERIAL_RX 16

const int total_num_servos = 18;
const float degPerUnit = 0.24;
const int legDevAngle = 15;
const float a1 = 37;
const float a2 = 63.54;
const float a3 = 150;

// move range limitation for hip, knee, and ankle servos in degrees
// since the hexapod enforces elbow up gait, the ankle servo can never be lower than 500 unit position, which is 0 degree
// due to float point error, lower limit of the ankle is set to a very small negative number (-0.1) instead of 0
const float range_limits[6] = { -30, 30, -90, 90, -0.1, 105 };

bool verbose = true;

HardwareSerial HardwareSerial(1);
LobotSerialServoControl BusServo(HardwareSerial);

void setup() {
  Serial.begin(115200);
  Serial.println("start...");
  HardwareSerial.begin(115200, SERIAL_8N1, SERVO_SERIAL_TX, SERVO_SERIAL_RX);
  BusServo.OnInit();
  delay(500);

  int available_servo_ids[total_num_servos];
  servo_check(available_servo_ids);
  for (int i = 0; i < total_num_servos; i++) {
    Serial.println(available_servo_ids[i]);
  }

  float* array = (float*)malloc(3 * sizeof(float));

  BusServo.lobotSerialServoOffsetWrite(3, -63);  // legs are 15 arcs, thus they require offsets to correct the endpoints to the calculated positions
  inverse_kinematics(array, a1 + a3, 0, a2);

  if (limitation_check(array, range_limits)) {
    Serial.println("--------------------");
    Serial.println("Moving");

    BusServo.LobotSerialServoMove(1, deg2unit(array[0]), 1000);
    BusServo.LobotSerialServoMove(2, deg2unit(array[1]), 1000);
    BusServo.LobotSerialServoMove(3, deg2unit(array[2]), 1000);
  }
  Serial.println(array[0]);
  Serial.println(array[1]);
  Serial.println(array[2]);

  Serial.println(deg2unit(array[0]));
  Serial.println(deg2unit(array[1]));
  Serial.println(deg2unit(array[2]));
  delay(3000);

  inverse_kinematics(array, a1 + a2 + a3, 0, 0);
  if (limitation_check(array, range_limits)) {
    Serial.println("--------------------");
    Serial.println("Moving");

    BusServo.LobotSerialServoMove(1, deg2unit(array[0]), 1000);
    BusServo.LobotSerialServoMove(2, deg2unit(array[1]), 1000);
    BusServo.LobotSerialServoMove(3, deg2unit(array[2]), 1000);
  }
  Serial.println(array[0]);
  Serial.println(array[1]);
  Serial.println(array[2]);

  Serial.println(deg2unit(array[0]));
  Serial.println(deg2unit(array[1]));
  Serial.println(deg2unit(array[2]));
  delay(3000);

  inverse_kinematics(array, a1 + a2, 0, -a3);
  if (limitation_check(array, range_limits)) {
    Serial.println("--------------------");
    Serial.println("Moving");

    BusServo.LobotSerialServoMove(1, deg2unit(array[0]), 1000);
    BusServo.LobotSerialServoMove(2, deg2unit(array[1]), 1000);
    BusServo.LobotSerialServoMove(3, deg2unit(array[2]), 1000);
  }
  Serial.println(array[0]);
  Serial.println(array[1]);
  Serial.println(array[2]);

  Serial.println(deg2unit(array[0]));
  Serial.println(deg2unit(array[1]));
  Serial.println(deg2unit(array[2]));
  delay(3000);
}

void loop() {}
