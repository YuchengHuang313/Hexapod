#include "ServoController.h"

#define GET_LOW_BYTE(A) (uint8_t)((A))
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))

ServoController::ServoController(HardwareSerialIMXRT &serial_port)
{
    SerialX = &serial_port;
    SerialX->begin(9600);
}

void ServoController::moveServos(Servo servos[], uint8_t num, uint16_t duration)
{
    uint8_t buf[128];
    if (duration <= 0)
    {
        return;
    }

    buf[0] = buf[1] = FRAME_HEADER;
    buf[2] = num * 3 + 5;
    buf[3] = CMD_SERVO_MOVE;
    buf[4] = num;
    buf[5] = GET_LOW_BYTE(duration);
    buf[6] = GET_HIGH_BYTE(duration);

    uint8_t index = 7;

    for (uint8_t i = 0; i < num; i++)
    {
        buf[index++] = servos[i].ID;
        buf[index++] = GET_LOW_BYTE(servos[i].Position);
        buf[index++] = GET_HIGH_BYTE(servos[i].Position);
    }
    SerialX->write(buf, buf[2] + 2);
}

bool ServoController::readServos(uint8_t IDs[], uint8_t num, uint16_t positions[])
{
    if (num < 1 || num > 32)
        return false;

    // 1) Build request: [0]=H, [1]=H, [2]=len, [3]=cmd, [4]=num, [5..]=IDs
    uint8_t len = num + 3;
    uint8_t req[5 + 32];
    req[0] = FRAME_HEADER;
    req[1] = FRAME_HEADER;
    req[2] = len;
    req[3] = CMD_MULT_SERVO_POS_READ;
    req[4] = num;
    memcpy(req + 5, IDs, num);

    // 2) Send it
    SerialX->write(req, 5 + num);
    SerialX->flush();

    // 3) Wait up to 100ms for a reply header 0x55,0x55
    unsigned long deadline = millis() + 100;
    uint8_t state = 0;
    while (millis() < deadline)
    {
        if (!SerialX->available())
            continue;
        uint8_t b = SerialX->read();
        if (state == 0 && b == FRAME_HEADER)
            state = 1;
        else if (state == 1 && b == FRAME_HEADER)
        {
            state = 2;
            break;
        }
        else
            state = 0;
    }
    if (state != 2)
        return false;

    // 4) Read the length byte
    while (millis() < deadline && !SerialX->available())
        ;
    if (!SerialX->available())
        return false;
    uint8_t respLen = SerialX->read(); // expected = num*3 + 3

    // 5) Read the rest of the frame (respLen bytes)
    uint8_t buf[64];
    uint8_t toRead = respLen;
    uint8_t got = 0;
    while (millis() < deadline && got < toRead)
    {
        if (SerialX->available())
        {
            got += SerialX->readBytes(buf + got, toRead - got);
        }
    }
    if (got < toRead)
        return false;

    // 6) Validate command and count
    if (buf[0] != CMD_MULT_SERVO_POS_READ)
        return false;
    if (buf[1] != num)
        return false;

    // 7) Extract each (ID, posL, posH) triple
    for (uint8_t i = 0; i < num; i++)
    {
        uint8_t idx = 2 + i * 3;
        uint8_t id = buf[idx + 0];
        uint8_t lo = buf[idx + 1];
        uint8_t hi = buf[idx + 2];
        if (id != IDs[i]) 
            return false; // mismatch
        positions[i] = (uint16_t(hi) << 8) | uint16_t(lo);
    }

    return true;
}
