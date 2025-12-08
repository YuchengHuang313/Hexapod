#include "HardwareSerial.h"
#include "Leg.h"
#include "Kinematics.h"
#include <Arduino.h>

// Only keep the static range_limits
double Leg::range_limits[6] = {
    -50.0, 50.0, // hip
    -95.0, 95.0, // knee
    -125.0, 5.0  // ankle
};

Leg::Leg(int leg_id, int hip_id, int knee_id, int ankle_id)
    : legId(leg_id),
      hipId(hip_id),
      kneeId(knee_id),
      ankleId(ankle_id),

      // Pick the right SerialX and hand it straight to busServo:
      //   busServo((leg_id <= 2)   ? Serial1
      //            : (leg_id <= 4) ? Serial2
      //                            : Serial3)
      busServo((leg_id <= 3) ? Serial1 : Serial2)

{
    // Begin whichever SerialX we chose:
    // if (leg_id <= 2)
    // {
    //     Serial.printf("Leg %d: using Serial1 at %dbps\n", legId, BAUDRATE);
    //     Serial1.begin(BAUDRATE);
    // }
    // else if (leg_id <= 4)
    // {
    //     Serial.printf("Leg %d: using Serial2 at %dbps\n", legId, BAUDRATE);
    //     Serial2.begin(BAUDRATE);
    // }
    // else
    // {
    //     Serial.printf("Leg %d: using Serial3 at %dbps\n", legId, BAUDRATE);
    //     Serial3.begin(BAUDRATE);
    // }
    // busServo.OnInit();
    if (leg_id <= 3)
    {
        Serial1.begin(BAUDRATE);
    }
    else
    {
        Serial2.begin(BAUDRATE);
    }
    busServo.OnInit();
}

bool Leg::leg_to_position(double x, double y, double z, int time_ms)
{
    // 1) Compute IK into thetas[] (in degrees)
    if (!inverse_kinematics(thetas, x, y, z,
                            HIP_TO_KNEE, KNEE_TO_ANKLE, ANKLE_TO_TIP))
    {
        Serial.println("Leg.cpp -> Error: IK failed");
        return false;
    }

    // 2) Physical range‐limit check (still in degrees)
    if (!leg_check_range_limits(thetas))
    {
        Serial.printf("Leg.cpp -> Leg %d Error: range limit exceeded at pos(%.1f,%.1f,%.1f)\n",
                      legId, x, y, z);
        return false;
    }

    // 3) Convert each theta → servo units (0…1000)
    int hipUnits = deg2unit(thetas[0]);
    int kneeUnits = deg2unit(thetas[1]);
    int ankleUnits = deg2unit(thetas[2]);

    // 4) **Inject your offset** into the ankle units:
    //    FOOT_UNIT_OFFSET was previously computed via ceil(15°/0.24°)
    ankleUnits += FOOT_UNIT_OFFSET;

    // 5) Clamp all three into the [0…1000] range the servo actually supports:
    hipUnits = constrain(hipUnits, 0, 1000);
    kneeUnits = constrain(kneeUnits, 0, 1000);
    ankleUnits = constrain(ankleUnits, 0, 1000);

    // 6) Send the three move commands
    busServo.LobotSerialServoMove(hipId, hipUnits, time_ms);
    busServo.LobotSerialServoMove(kneeId, kneeUnits, time_ms);
    busServo.LobotSerialServoMove(ankleId, ankleUnits, time_ms);

    return true;
}

void Leg::leg_unload_all()
{
    Serial.printf("Leg.cpp -> Unloading servos for Leg %d: hip=%d, knee=%d, ankle=%d\n",
                  legId, hipId, kneeId, ankleId);

    // Unload each servo individually with delay and error checking
    busServo.LobotSerialServoUnload(hipId);
    busServo.LobotSerialServoUnload(kneeId);
    busServo.LobotSerialServoUnload(ankleId);
    delay(100);
}

bool Leg::leg_read_position(double pos[3])
{
    // 1) Read raw servo units back from the controller
    int rawHip = busServo.LobotSerialServoReadPosition(hipId);
    int rawKnee = busServo.LobotSerialServoReadPosition(kneeId);
    int rawAnkle = busServo.LobotSerialServoReadPosition(ankleId);

    // 2) Compensate the ankle reading by subtracting the offset
    int compAnkle = rawAnkle - FOOT_UNIT_OFFSET;
    // (if you want to guard against underflow/overflow, you can clamp here:
    //  compAnkle = constrain(compAnkle, 0, 1000); )

    // 3) Convert each back into an angle in degrees (500→0° center)
    double angHip = (rawHip - 500) * DEGREE_PER_UNIT;
    double angKnee = (rawKnee - 500) * DEGREE_PER_UNIT;
    double angAnkle = (compAnkle - 500) * DEGREE_PER_UNIT;

    // 4) Run forward kinematics to get Cartesian position
    bool fk_check = forward_kinematics(pos, range_limits,
                                       angHip, angKnee, angAnkle,
                                       HIP_TO_KNEE, KNEE_TO_ANKLE, ANKLE_TO_TIP);

    // 5) Print it out
    Serial.printf("Leg.cpp -> Actual Leg %d: raw=[%d,%d,%d]  adj ankle=%d  x=%.2f y=%.2f z=%.2f\n",
                  legId,
                  rawHip, rawKnee, rawAnkle,
                  compAnkle,
                  pos[0], pos[1], pos[2]);

    return fk_check;
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