#include "ServoController.h"

#define GET_LOW_BYTE(A) (uint8_t)((A))
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))

ServoController::ServoController(HardwareSerialIMXRT &serial_port)
{
    SerialX = &serial_port;
    SerialX->begin(9600);
    SerialX->setTimeout(5);
}

void ServoController::moveServos(Servo servos[], uint8_t num, uint16_t duration)
{
    size_t send_buf_size = 7 + num * 3;
    uint8_t buf[send_buf_size];
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
    SerialX->write(buf, send_buf_size);
    SerialX->flush();
}

bool ServoController::readServos(Servo servos[], uint8_t num)
{
    for (int i = 0; i < num; i++)
    {
        if (servos[i].ID <= 0 || servos[i].ID >= 32)
        {
            Serial.printf("ServoController failure: ID = %d\n", servos[i].ID);
            return false;
        }
    }

    // sending read command to the controller
    size_t send_buf_size = 5 + num;
    uint8_t buf[send_buf_size];
    buf[0] = buf[1] = 0x55;
    buf[2] = num + 3;
    buf[3] = CMD_MULT_SERVO_POS_READ;
    buf[4] = num;
    for (int i = 0; i < num; i++)
    {
        buf[i + 5] = servos[i].ID;
    }
    SerialX->write(buf, send_buf_size);
    SerialX->flush();

    // reading data came back
    size_t resp_buf_size = 5 + num * 3;
    uint8_t resp[resp_buf_size];
    size_t num_bytes = SerialX->readBytes(resp, resp_buf_size);

    if (num_bytes != resp_buf_size)
    {
        Serial.printf("ServoController failure: Expecting to read %d bytes, Actually %d bytes\n", resp_buf_size, num_bytes);
        return false;
    }
    else
    {
        // Serial.print("reading Rx: { ");
        // for (size_t i = 0; i < num_bytes; i++)
        // {
        //     Serial.printf("0x%02x ", resp[i]);
        // }
        // Serial.println("} done reading");

        // interpret data
        for (uint8_t j = 0; j < num; j++)
        {
            uint8_t idx = 5 + j * 3; // start of this servo’s chunk
            servos[j].ID = resp[idx];
            uint8_t low_byte = resp[idx + 1];
            uint8_t high_byte = resp[idx + 2];
            servos[j].Position = BYTE_TO_HW(high_byte, low_byte);
        }
    }
    SerialX->clear();
    return true;
}

void ServoController::unloadServos(Servo servos[], uint8_t num)
{
    size_t send_buf_size = 5 + num;
    uint8_t buf[send_buf_size];
    buf[0] = buf[1] = 0x55;
    buf[2] = num + 3;
    buf[3] = CMD_MULT_SERVO_UNLOAD;
    buf[4] = num;

    for (size_t i = 0; i < num; i++)
    {
        buf[i + 5] = servos[i].ID;
    }
    SerialX->write(buf, send_buf_size);
    SerialX->flush();
}
