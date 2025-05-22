#include "esp32-hal.h"
#include "HardwareSerial.h"
#include "Leg.h"
#include "Kinematics.h"
#include <Arduino.h>

HardwareSerial Leg::hardwareSerial(1);
LobotSerialServoControl Leg::busServo(Leg::hardwareSerial);
double Leg::range_limits[6] = {
    -46.0, 46.0, // hip
    -91.0, 91.0, // knee
    -121.0, 1.0  // ankle
};

Leg::Leg(int leg_id, int hip_id, int knee_id, int ankle_id)
{
    legId = leg_id;
    hipId = hip_id;
    kneeId = knee_id;
    ankleId = ankle_id;
    hardwareSerial.begin(BAUDRATE, SERIAL_8N1, leg_id > 3 ? 17 : 5, leg_id > 3 ? 16 : 4);
    busServo.OnInit();
    busServo.lobotSerialServoOffsetWrite(ankleId, FOOT_UNIT_OFFSET);
}

bool Leg::leg_to_position(double x, double y, double z, int time_ms)
{
    if (!inverse_kinematics(thetas, x, y, z,
                            HIP_TO_KNEE, KNEE_TO_ANKLE, ANKLE_TO_TIP))
    {
        Serial.println("Error: IK failed");
        return false;
    }
    if (!leg_check_range_limits(thetas))
    {
        Serial.println("Error: range limit");
        return false;
    }
    busServo.LobotSerialServoMove(hipId, deg2unit(thetas[0]), time_ms);
    busServo.LobotSerialServoMove(kneeId, deg2unit(thetas[1]), time_ms);
    busServo.LobotSerialServoMove(ankleId, deg2unit(thetas[2]), time_ms);
    delay(time_ms);
    return true;
}

/**
 * @brief point-to-point move in straight line with adaptive timing
 * @param start_x start x
 * @param start_y start y
 * @param start_z start z
 * @param end_x   end x
 * @param end_y   end y
 * @param end_z   end z
 * @return true if sweep completed (or first IK failure encountered)
 */
bool Leg::leg_p2p(double sx, double sy, double sz,
                  double ex, double ey, double ez)
{
    // compute delta and distance
    double dx = ex - sx;
    double dy = ey - sy;
    double dz = ez - sz;
    double dist = sqrt(dx * dx + dy * dy + dz * dz);
    // step count
    int steps = int(dist / STEP_SIZE + 0.5);
    if (steps < 1)
        steps = 1;
    int ms_per_move = TOTAL_MS / steps;
    int uart_pause = (ms_per_move / 2 >= 1 ? ms_per_move / 2 : 1);
    Serial.printf("leg_p2p: dist=%.1f steps=%d, move=%dms, pause=%dms\n",
                  dist, steps, ms_per_move, uart_pause);
    // sweep
    for (int i = 0; i <= steps; ++i)
    {
        double t = double(i) / steps;
        double x = sx + dx * t;
        double y = sy + dy * t;
        double z = sz + dz * t;
        // optional dead-zone clamp
        // if (fabs(x) < STEP_SIZE)
        //     x = (x >= 0 ? STEP_SIZE : -STEP_SIZE);
        // if (fabs(y) < STEP_SIZE)
        //     y = (y >= 0 ? STEP_SIZE : -STEP_SIZE);
        // if (fabs(z) < STEP_SIZE)
        //     z = (z >= 0 ? STEP_SIZE : -STEP_SIZE);
        if (!leg_to_position(x, y, z, ms_per_move))
        {
            Serial.printf(" IK fail @%3d: x=%.1f y=%.1f z=%.1f\n",
                          i, x, y, z);
            return false;
        }
        delay(uart_pause);
    }
    // wait final
    delay(ms_per_move + uart_pause + 100);
    return true;
}

void Leg::leg_unload_all()
{
    for (int id = hipId; id < hipId + TOTAL_LEG_SERVOS; ++id)
        busServo.LobotSerialServoUnload(id);
}

void Leg::leg_read_position()
{
    double ang[3], pos[3];
    ang[0] = (busServo.LobotSerialServoReadPosition(hipId) - 500) * DEGREE_PER_UNIT;
    ang[1] = (busServo.LobotSerialServoReadPosition(kneeId) - 500) * DEGREE_PER_UNIT;
    ang[2] = (busServo.LobotSerialServoReadPosition(ankleId) - 500) * DEGREE_PER_UNIT;
    forward_kinematics(pos, range_limits,
                       ang[0], ang[1], ang[2],
                       HIP_TO_KNEE, KNEE_TO_ANKLE, ANKLE_TO_TIP);
    Serial.printf("Actual Leg %d: x=%.2f y=%.2f z=%.2f\n",
                  legId, pos[0], pos[1], pos[2]);
}

int Leg::leg_get_id() { return legId; }
int Leg::leg_get_hip_id() { return hipId; }
int Leg::leg_get_knee_id() { return kneeId; }
int Leg::leg_get_ankle_id() { return ankleId; }

bool Leg::leg_check_range_limits(double *input_angles)
{
    return (input_angles[0] >= range_limits[0] && input_angles[0] <= range_limits[1] &&
            input_angles[1] >= range_limits[2] && input_angles[1] <= range_limits[3] &&
            input_angles[2] >= range_limits[4] && input_angles[2] <= range_limits[5]);
}
