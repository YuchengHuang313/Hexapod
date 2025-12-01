// Hexapod.cpp

#include "Hexapod.h"
#include "Leg.h"

// Constructor to initialize legs with appropriate servo IDs
Hexapod::Hexapod() : legs{
                         {1, 1, 2, 3},    // Leg 1 with servo IDs 1, 2, 3
                         {2, 4, 5, 6},    // Leg 2 with servo IDs 4, 5, 6
                         {3, 7, 8, 9},    // Leg 3 with servo IDs 7, 8, 9
                         {4, 10, 11, 12}, // Leg 4 with servo IDs 10, 11, 12
                         {5, 13, 14, 15}, // Leg 5 with servo IDs 13, 14, 15
                         {6, 16, 17, 18}  // Leg 6 with servo IDs 16, 17, 18
                     }
{
    // Any additional initialization if necessary
}

// Method to move the hexapod to a given position
bool Hexapod::hexapod_to_position(double x, double y, double z)
{

    return false;
}

bool Hexapod::hexapod_to_position_mirror(double target_pos[3])
{
    // 1) grab each leg’s current tip position
    double start_pos[6][3];
    if (!hexapod_read_all_leg_pos(start_pos))
    {
        Serial.println("Hexapod.cpp -> hexapod_to_position_mirror: failed to read all leg positions");
        return false;
    }

    // 2) compute per-leg deltas and average length
    double distances[6][3];
    double total_length = 0;
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            distances[i][j] = target_pos[j] - start_pos[i][j];
        }
        double d = sqrt(
            sq(distances[i][0]) +
            sq(distances[i][1]) +
            sq(distances[i][2]));
        total_length += d;
    }
    double avg_length = total_length / 6.0;

    // 3) plan steps
    int steps = int(avg_length / STEP_SIZE + 0.5);
    if (steps < 1)
        steps = 1;
    int base_ms_per_move = TOTAL_MS / steps;
    Serial.printf("Hexapod.cpp -> hexapod_to_position_mirror: avg_dist=%.1f steps=%d base_ms_step=%d\n",
                  avg_length, steps, base_ms_per_move);

    // 4) sweep each leg along its own vector with cubic time scaling
    for (int s = 0; s <= steps; ++s)
    {
        double t = double(s) / steps;
        double t_ratio = time_scaling(t);

        int ms_per_move = max(base_ms_per_move, int(base_ms_per_move * t_ratio));
        Serial.printf("  step %d/%d: t=%.3f t_ratio=%.3f ms_per_move=%d\n",
                      s, steps, t, t_ratio, ms_per_move);
        bool all_ok = true;
        for (int leg = 0; leg < 6; ++leg)
        {
            double x = start_pos[leg][0] + distances[leg][0] * t;
            double y = start_pos[leg][1] + distances[leg][1] * t;
            double z = start_pos[leg][2] + distances[leg][2] * t;
            all_ok &= legs[leg].leg_to_position(x, y, z, ms_per_move);
        }
        delay(ms_per_move);
    }
    delay(50);
    return true;
}

bool Hexapod::hexapod_read_all_leg_pos(double leg_servo_pos[6][3])
{
    bool success = true;
    for (int i = 0; i < 6; i++)
    {
        success &= legs[i].leg_read_position(leg_servo_pos[i]);
    }
    return success;
}

void Hexapod::hexapod_unload()
{
    for (int i = 0; i < 6; i++)
    {
        legs[i].leg_unload_all();
    }
}

double Hexapod::time_scaling(double t) const
{
    // given t in [0, 1] but we are scaling directly on the time from one
    // position to the next, we want to ease in and out. Thus the time scaling
    // function is just a quadratic ease-in-out curve.

    return max(1.0, 20.0 * (t - 0.5) * (t - 0.5));
}
