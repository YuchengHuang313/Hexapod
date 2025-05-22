#ifndef LEG_H
#define LEG_H

#include "LobotSerialServoControl.h"
#include <math.h>

class Leg
{
private:
    // instance fields
    int legId;
    int hipId;
    int kneeId;
    int ankleId;
    double thetas[3] = {0, 0, 0};

    // static fields
    static constexpr int TOTAL_LEG_SERVOS = 3;
    static constexpr int FOOT_DEV_ANGLE = 15;
    static constexpr double DEGREE_PER_UNIT = 0.24;
    static constexpr int FOOT_UNIT_OFFSET = ceil(FOOT_DEV_ANGLE / DEGREE_PER_UNIT);
    static double range_limits[6];

    // serial config
    static HardwareSerial hardwareSerial;
    static constexpr int BAUDRATE = 115200;
    static constexpr int SERVO_SERIAL_TX = 5; // 17
    static constexpr int SERVO_SERIAL_RX = 4; // 16

public:
    // link lengths
    static constexpr float HIP_TO_KNEE = 37.0f;
    static constexpr float KNEE_TO_ANKLE = 63.54f;
    static constexpr float ANKLE_TO_TIP = 200.0f;

    // sweep parameters
    static constexpr int TOTAL_MS = 1000;    // ms per p2p sweep
    static constexpr float STEP_SIZE = 1.0f; // mm per step

    static LobotSerialServoControl busServo;

    Leg(int leg_id, int hip_id, int knee_id, int ankle_id);
    bool leg_to_position(double x, double y, double z, int time_ms);
    bool leg_p2p(double start_x,
                 double start_y,
                 double start_z,
                 double end_x,
                 double end_y,
                 double end_z);
    void leg_unload_all();
    void leg_read_position();
    int leg_get_id();
    int leg_get_hip_id();
    int leg_get_knee_id();
    int leg_get_ankle_id();
    bool leg_check_range_limits(double *input_angles);
};

#endif // LEG_H