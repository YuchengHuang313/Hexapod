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
        double t_eased = cubic_ease_in_out(t);

        // Scale ms_per_move: starts high, decreases as motion progresses
        // Uses (1 - t_eased)^2 for aggressive acceleration curve
        // At t=0: multiplier = 1.0 (normal speed)
        // At t=1: multiplier = 0.0 (fastest, limited to 1ms minimum)
        double multiplier = (1.0 - t_eased) * (1.0 - t_eased); // quadratic falloff
        int ms_per_move = int(base_ms_per_move * multiplier + 0.5);
        // Clamp to minimum of 1ms (manufacturer requirement)
        if (ms_per_move < 1)
            ms_per_move = 1;

        bool all_ok = true;
        for (int leg = 0; leg < 6; ++leg)
        {
            double x = start_pos[leg][0] + distances[leg][0] * t;
            double y = start_pos[leg][1] + distances[leg][1] * t;
            double z = start_pos[leg][2] + distances[leg][2] * t;
            all_ok &= legs[leg].leg_to_position(x, y, z, ms_per_move);
        }
        // if (!all_ok)
        // {
        //     Serial.printf("Hexapod.cpp -> hexapod_to_position_mirror: move failed @ step %d\n", s);
        //     return false;
        // }
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

double Hexapod::cubic_ease_in_out(double t) const
{
    // Cubic ease-in-out function
    // Returns a value from 0 to 1, with cubic acceleration/deceleration
    // Starts slow, accelerates in middle, slows down at end
    if (t < 0.5)
    {
        // First half: ease in (cubic acceleration)
        return 4.0 * t * t * t;
    }
    else
    {
        // Second half: ease out (cubic deceleration)
        double t_shifted = t - 1.0;
        return 1.0 + 4.0 * t_shifted * t_shifted * t_shifted;
    }
}
