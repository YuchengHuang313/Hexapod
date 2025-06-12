#ifndef LOBOTSERVOCONTROLLER_H
#define LOBOTSERVOCONTROLLER_H

#include <Arduino.h>

// 发送部分的指令
#define FRAME_HEADER 0x55            // 帧头
#define CMD_SERVO_MOVE 0x03          // 舵机移动指令
#define CMD_ACTION_GROUP_RUN 0x06    // 运行动作组指令
#define CMD_ACTION_GROUP_STOP 0x07   // 停止动作组运行指令
#define CMD_ACTION_GROUP_SPEED 0x0B  // 设置动作组运行速度指令
#define CMD_GET_BATTERY_VOLTAGE 0x0F // 获得电池电压指令
#define CMD_MULT_SERVO_UNLOAD 0x14   // 多个舵机掉电指令
#define CMD_MULT_SERVO_POS_READ 0x15 // 读取多个舵机的角度位置指令

// 接收部分的指令
#define BATTERY_VOLTAGE 0x0F       // 电池电压
#define ACTION_GROUP_RUNNING 0x06  // 动作组被运行
#define ACTION_GROUP_STOPPED 0x07  // 动作组被停止
#define ACTION_GROUP_COMPLETE 0x08 // 动作组完成

struct Servo
{                      // 舵机ID和位置结构体
    uint8_t ID;        // 舵机ID
    uint16_t Position; // 舵机数据
};

class ServoController
{
public:
    HardwareSerialIMXRT *SerialX;
    ServoController(HardwareSerialIMXRT &serial_port);
    void moveServos(Servo servos[], uint8_t num, uint16_t duration);
    bool readServos(uint8_t IDs[], uint8_t num, uint16_t positions[]);
};
#endif