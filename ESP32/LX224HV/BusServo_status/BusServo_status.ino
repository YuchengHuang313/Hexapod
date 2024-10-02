#include "LobotSerialServoControl.h"  // 导入库文件

// 读取总线舵机信息

#define SERVO_SERIAL_TX 17
#define SERVO_SERIAL_RX 16

HardwareSerial HardwareSerial(1);
LobotSerialServoControl BusServo(HardwareSerial);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);                                                        // 设置串口波特率
  Serial.println("start...");                                                  // 串口打印"start..."
  HardwareSerial.begin(115200, SERIAL_8N1, SERVO_SERIAL_TX, SERVO_SERIAL_RX);  // baud rate, protocol, slave tx pin (master rx pin), slave serial rx pin (master tx pin).
  BusServo.OnInit();                                                           // 初始化总线舵机库
  delay(500);                                                                  // 延时500毫秒

  BusServo.LobotSerialServoMove(1, 300, 1000);
  Serial.println("init");
  delay(3000);

  double time = millis();
  for (int i = 300; i <= 700; i++) {
    BusServo.LobotSerialServoMove(1, i, 10);
    while (i % 100 == 0 && abs(BusServo.LobotSerialServoReadPosition(1) - i) > 25) {
      continue;
    }
  }
  delay(1000);
  if (BusServo.LobotSerialServoReadPosition(1) >= 675 && BusServo.LobotSerialServoReadPosition(1) <= 725) {
    Serial.printf("Expected 700 reached %i\n", BusServo.LobotSerialServoReadPosition(1));
  } else {
    Serial.println("FAILED 700");
  }
  double time1 = millis();
  Serial.println(time1-time-1000);
  delay(1000);

  double time2 = millis();
  for (int i = 700; i >= 300; i--) {
    BusServo.LobotSerialServoMove(1, i, 10);
    while (i % 100 == 0 && abs(BusServo.LobotSerialServoReadPosition(1) - i) > 25) {
      continue;
    }
  }
  delay(1000);
  if (BusServo.LobotSerialServoReadPosition(1) >= 275 && BusServo.LobotSerialServoReadPosition(1) <= 325) {
    Serial.printf("Expected 300 reached %i\n", BusServo.LobotSerialServoReadPosition(1));
  } else {
    Serial.println("FAILED 300");
  }
  double time3 = millis();
  Serial.println(time3-time2-1000);
  delay(1000);

  BusServo.LobotSerialServoUnload(1);
  Serial.println("Done");
}

void loop() {
}
