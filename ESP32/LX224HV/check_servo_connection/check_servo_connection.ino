#include "LobotSerialServoControl.h"
#define SERVO_SERIAL_TX 17
#define SERVO_SERIAL_RX 16
#define total_num_servos 18

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
}

void loop() {}

void servo_check(int *available_servo_ids) {
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
