#ifndef LOBOTSERVOCONTROLLER_H
#define LOBOTSERVOCONTROLLER_H

#include <Arduino.h>

// 发送部分的指令
#define FRAME_HEADER 0x55            // 帧头
#define CMD_SERVO_MOVE 0x03          // 舵机移动指令
#define CMD_GET_BATTERY_VOLTAGE 0x0F // 获得电池电压指令
#define CMD_MULT_SERVO_UNLOAD 0x14   // 多个舵机掉电指令
#define CMD_MULT_SERVO_POS_READ 0x15 // 读取多个舵机的角度位置指令

// 接收部分的指令
#define BATTERY_VOLTAGE 0x0F // 电池电压

struct Servo
{
    uint8_t ID;
    uint16_t Position;
};

class ServoController
{
public:
    HardwareSerialIMXRT *SerialX;
    ServoController(HardwareSerialIMXRT &serial_port);
    void moveServos(Servo servos[], uint8_t num, uint16_t duration);
    bool readServos(Servo servos[], uint8_t num);
    void unloadServos(Servo servos[], uint8_t num);
};
#endif