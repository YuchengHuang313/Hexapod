#include <Arduino.h>
#include "LobotSerialServoControl.h"
#include "ServoController.h"

ServoController controller(Serial1);

Servo servo_command1[] = {1, 500};
Servo servo_command2[] = {1, 0};

void setup()
{
    // Initialize USB Serial for debugging (optional)
    Serial.begin(115200);
    delay(1000);
}

void loop()
{
    uint8_t readPos[] = {0x55, 0x55, 0x04, 0x15, 0x01, 0x01};
    Serial.println("Sent command 1");
    // Serial1.write(moveTo1000, 10);
    controller.moveServos(servo_command1, 1, 1500);
    delay(2000);
    Serial1.write(readPos, 6);
    Serial1.flush();
    delay(10);
    Serial.print("reading data 1: {");
    while (Serial1.available())
    {
        Serial.printf(" 0x%02x ", Serial1.read());
        delay(1);
    }
    Serial.println("} done reading");

    Serial.println("Sent command 2");
    controller.moveServos(servo_command2, 1, 1500);
    // Serial1.write(moveTo0, 10);
    delay(2000);
    Serial1.write(readPos, 6);
    Serial1.flush();
    delay(10);
    Serial.print("reading data 2: {");
    while (Serial1.available())
    {
        Serial.printf(" 0x%02x ", Serial1.read());
        delay(1);
    }
    Serial.println("} done reading");
}

// #include <Arduino.h>
// #include "Leg.h"

// void setup()
// {
//     Serial.begin(115200);
//     delay(500);
//     Serial.println("=== Leg Movement Test Suite (Parallel Multi-leg Version) ===");
//     // Instantiate legs
//     Leg leg1(1, 1, 2, 3);
//     Leg leg2(2, 4, 5, 6);
//     Leg leg3(3, 7, 8, 9);
//     Leg leg4(4, 10, 11, 12);
//     Leg leg5(5, 13, 14, 15);
//     Leg leg6(6, 16, 17, 18);
//     delay(500);

//     // Neutral leg position
//     float x0 = Leg::HIP_TO_KNEE + Leg::ANKLE_TO_TIP;
//     float z0 = Leg::KNEE_TO_ANKLE;
//     float y_start = 150.0f;
//     float y_end = -150.0f;
//     // Move all legs to the same starting position
//     Serial.println("\n1) Move all legs to starting position");
//     leg1.leg_to_position(x0, y_start, z0, Leg::TOTAL_MS);
//     leg2.leg_to_position(x0, y_start, z0, Leg::TOTAL_MS);
//     leg3.leg_to_position(x0, y_start, z0, Leg::TOTAL_MS);
//     leg4.leg_to_position(x0, y_start, z0, Leg::TOTAL_MS);
//     leg5.leg_to_position(x0, y_start, z0, Leg::TOTAL_MS);
//     leg6.leg_to_position(x0, y_start, z0, Leg::TOTAL_MS);
//     delay(Leg::TOTAL_MS + 100);

//     // Compute total distance and steps
//     double dx = 0.0;
//     double dy = y_start - y_end;
//     double dz = 0.0;
//     double dist = sqrt(dx * dx + dy * dy + dz * dz);
//     int steps = int(dist / Leg::STEP_SIZE + 0.5);
//     if (steps < 1)
//         steps = 1;
//     int ms_per_move = Leg::TOTAL_MS / steps;
//     int uart_pause = (ms_per_move / 2 >= 1 ? ms_per_move / 2 : 1);
//     Serial.printf("Parallel sweep: dist=%.1f steps=%d move=%dms pause=%dms\n",
//                   dist, steps, ms_per_move, uart_pause);

//     // Sweep all three legs together
//     for (int i = 0; i <= steps; ++i)
//     {
//         double t = double(i) / steps;
//         double x = x0;
//         double y = y_start + (y_end - y_start) * t;
//         double z = z0;

//         bool success =
//             leg1.leg_to_position(x, y, z, ms_per_move) &&
//             leg2.leg_to_position(x, y, z, ms_per_move) &&
//             leg3.leg_to_position(x, y, z, ms_per_move) &&
//             leg4.leg_to_position(x, y, z, ms_per_move) &&
//             leg5.leg_to_position(x, y, z, ms_per_move) &&
//             leg6.leg_to_position(x, y, z, ms_per_move);

//         if (!success)
//         {
//             Serial.printf("Parallel move failed @ step %d: x=%.1f y=%.1f z=%.1f\n", i, x, y, z);
//             break;
//         }

//         delay(uart_pause);
//     }

//     delay(ms_per_move + uart_pause + 100);

//     leg1.leg_read_position();
//     leg2.leg_read_position();
//     leg3.leg_read_position();
//     leg4.leg_read_position();
//     leg5.leg_read_position();
//     leg6.leg_read_position();

//     leg1.leg_unload_all();
//     leg2.leg_unload_all();
//     leg3.leg_unload_all();
//     leg4.leg_unload_all();
//     leg5.leg_unload_all();
//     leg6.leg_unload_all();
//     Serial.println("=== Test Complete ===");
// }

// void loop()
// {
//     // Nothing here
// }
