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

bool Hexapod::hexapod_to_position_mirror(double target_pos[3], TimeScalingFunc scaling_func)
{
    // 1) grab each leg's current tip position
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
        double t_ratio = (this->*scaling_func)(t);

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

double Hexapod::time_scaling_quad(double t) const
{
    // given t in [0, 1] but we are scaling directly on the time from one
    // position to the next, we want to ease in and out. Thus the time scaling
    // function is just a quadratic ease-in-out curve.

    return max(1.0, 50 * (t - 0.5) * (t - 0.5));
}

double Hexapod::time_scaling_trapezoidal(double t) const
{
    // given t in [0, 1] but we are scaling directly on the time from one
    // position to the next, we want to ease in and out. Thus the time scaling
    // function is just a trapezoidal curve.

    if (t < 0.1)
    {
        return -7 * (5 * t + 1); // ease in
    }
    else if (t > 0.9)
    {
        return 7 * (5 * t - 4); // ease out
    }
    else
    {
        return 1.0; // constant speed
    }
}

bool Hexapod::tripod_move(double direction[3], double step_height, TimeScalingFunc scaling_func)
{
    // Tripod gait: legs 1,3,5 move together, then legs 2,4,6 move together
    // Each phase has two sub-phases:
    //   - Curve phase (swing): leg lifts, moves forward in arc, lands
    //   - Line phase (stance): leg stays on ground, pushes backward

    // Normalize direction vector in XY plane
    double dir_mag = sqrt(direction[0] * direction[0] + direction[1] * direction[1]);
    if (dir_mag < 0.001)
    {
        Serial.println("Hexapod.cpp -> tripod_move: invalid direction vector");
        return false;
    }

    double dir_normalized[2] = {
        direction[0] / dir_mag,
        direction[1] / dir_mag};

    if (step_height <= 0)
    {
        Serial.println("Hexapod.cpp -> tripod_move: step_height must be positive");
        return false;
    }

    // direction[2] is the ground contact Z height (negative value, e.g., -150)
    float ground_z = direction[2];
    if (ground_z > 0)
    {
        Serial.println("Hexapod.cpp -> tripod_move: ground Z must be negative (legs extend downward from z=0)");
        return false;
    }

    // Get workspace parameters at the ground contact height
    float center_x_ground, center_y_ground, radius_ground;
    float lookup_z_ground = ground_z;

    if (!getWorkspaceAtZ(lookup_z_ground, center_x_ground, center_y_ground, radius_ground))
    {
        Serial.printf("Hexapod.cpp -> tripod_move: no workspace at ground z=%.1f\n", lookup_z_ground);
        return false;
    }

    // Also check workspace at lifted height (during swing phase)
    float center_x_lifted, center_y_lifted, radius_lifted;
    float lookup_z_lifted = ground_z + step_height; // Lifted Z (closer to 0)

    if (!getWorkspaceAtZ(lookup_z_lifted, center_x_lifted, center_y_lifted, radius_lifted))
    {
        Serial.printf("Hexapod.cpp -> tripod_move: no workspace at lifted z=%.1f\n", lookup_z_lifted);
        return false;
    }

    // Use the SMALLER radius to ensure legs stay within workspace at both heights
    float safe_radius = min(radius_ground, radius_lifted);

    // Stride = 2 * radius for full diameter movement
    // But use a safety factor since rotated legs have asymmetric reach
    double stride_length = safe_radius * 2.0 * 1; 

    Serial.printf("Hexapod.cpp -> tripod_move: ground_z=%.1f step_height=%.1f\n", ground_z, step_height);
    Serial.printf("  Ground workspace: center(%.1f,%.1f) r=%.1f\n", center_x_ground, center_y_ground, radius_ground);
    Serial.printf("  Lifted workspace: center(%.1f,%.1f) r=%.1f\n", center_x_lifted, center_y_lifted, radius_lifted);
    Serial.printf("  Safe radius=%.1f, stride=%.1f\n", safe_radius, stride_length);

    // Get current leg positions
    double current_pos[6][3];
    if (!hexapod_read_all_leg_pos(current_pos))
    {
        Serial.println("Hexapod.cpp -> tripod_move: failed to read positions");
        return false;
    }

    // Calculate absolute extreme positions for full diameter movement
    double half_stride = stride_length / 2.0;
    double forward_extreme[3] = {
        center_x_ground + dir_normalized[0] * half_stride,
        center_y_ground + dir_normalized[1] * half_stride,
        ground_z};
    double backward_extreme[3] = {
        center_x_ground - dir_normalized[0] * half_stride,
        center_y_ground - dir_normalized[1] * half_stride,
        ground_z};

    // Calculate number of steps for smooth motion
    int steps = int(stride_length / STEP_SIZE + 0.5);
    if (steps < 1)
        steps = 1;
    int ms_per_step = TOTAL_MS / steps;

    // PHASE 1: Legs 1,3,5 swing (curve), Legs 2,4,6 stance (line)
    Serial.println("Phase 1: Legs 1,3,5 swing");
    Serial.printf("DEBUG: Global dir=[%.3f, %.3f], stride=%.1f\n", dir_normalized[0], dir_normalized[1], stride_length);
    Serial.printf("  Backward extreme: [%.1f, %.1f, %.1f]\n", backward_extreme[0], backward_extreme[1], backward_extreme[2]);
    Serial.printf("  Forward extreme: [%.1f, %.1f, %.1f]\n", forward_extreme[0], forward_extreme[1], forward_extreme[2]);

    for (int s = 0; s <= steps; ++s)
    {
        double t = double(s) / steps;
        double t_scaled = (this->*scaling_func)(t);

        int ms_per_move = max(ms_per_step, int(ms_per_step * t_scaled));

        // Legs 1, 3, 5: curve phase (swing) - backward extreme to forward extreme
        for (int leg_idx : {0, 2, 4})
        {
            double target[3];
            // X,Y: interpolate from backward to forward extreme
            target[0] = backward_extreme[0] + t * (forward_extreme[0] - backward_extreme[0]);
            target[1] = backward_extreme[1] + t * (forward_extreme[1] - backward_extreme[1]);
            // Z: parabolic arc - lift in middle of swing
            target[2] = ground_z + 4.0 * step_height * t * (1.0 - t);

            if (s == 0 || s == steps / 2 || s == steps) // Debug first, middle, and last step
            {
                Serial.printf("DEBUG Leg %d step %d: t=%.3f, target=[%.1f,%.1f,%.1f]\n",
                              leg_idx + 1, s, t, target[0], target[1], target[2]);
            }

            bool result = legs[leg_idx].leg_to_position(target[0], target[1], target[2], ms_per_move);
            if (!result && s < 5) // Only show first few errors
            {
                Serial.printf("ERROR: Leg %d failed at step %d, t=%.3f, target=[%.1f,%.1f,%.1f]\n",
                              leg_idx + 1, s, t, target[0], target[1], target[2]);
            }
        }

        // Legs 2, 4, 6: line phase (stance) - forward extreme to backward extreme (push body forward)
        for (int leg_idx : {1, 3, 5})
        {
            double target[3];
            // Interpolate from forward to backward extreme at ground level
            target[0] = forward_extreme[0] + t * (backward_extreme[0] - forward_extreme[0]);
            target[1] = forward_extreme[1] + t * (backward_extreme[1] - forward_extreme[1]);
            target[2] = ground_z;
            legs[leg_idx].leg_to_position(target[0], target[1], target[2], ms_per_move);
        }

        delay(ms_per_move);
    }

    // CRITICAL: Update current positions after phase 1 to avoid drift
    if (!hexapod_read_all_leg_pos(current_pos))
    {
        Serial.println("Hexapod.cpp -> tripod_move: failed to read positions after phase 1");
        return false;
    }

    // PHASE 2: Legs 2,4,6 swing (curve), Legs 1,3,5 stance (line)
    Serial.println("Phase 2: Legs 2,4,6 swing");
    for (int s = 0; s <= steps; ++s)
    {
        double t = double(s) / steps;
        double t_scaled = (this->*scaling_func)(t);

        int ms_per_move = max(ms_per_step, int(ms_per_step * t_scaled));

        // Legs 2, 4, 6: curve phase (swing) - backward extreme to forward extreme
        for (int leg_idx : {1, 3, 5})
        {
            double target[3];
            // X,Y: interpolate from backward to forward extreme
            target[0] = backward_extreme[0] + t * (forward_extreme[0] - backward_extreme[0]);
            target[1] = backward_extreme[1] + t * (forward_extreme[1] - backward_extreme[1]);
            // Z: parabolic arc - lift in middle of swing
            target[2] = ground_z + 4.0 * step_height * t * (1.0 - t);
            legs[leg_idx].leg_to_position(target[0], target[1], target[2], ms_per_move);
        }

        // Legs 1, 3, 5: line phase (stance) - forward extreme to backward extreme (push body forward)
        for (int leg_idx : {0, 2, 4})
        {
            double target[3];
            // Interpolate from forward to backward extreme at ground level
            target[0] = forward_extreme[0] + t * (backward_extreme[0] - forward_extreme[0]);
            target[1] = forward_extreme[1] + t * (backward_extreme[1] - forward_extreme[1]);
            target[2] = ground_z;
            legs[leg_idx].leg_to_position(target[0], target[1], target[2], ms_per_move);
        }

        delay(ms_per_move);
    }

    // Return all legs to center position to prevent drift across multiple calls
    Serial.println("Returning legs to center position...");
    double center_pos[3] = {center_x_ground, center_y_ground, ground_z};

    for (int leg_idx = 0; leg_idx < 6; leg_idx++)
    {
        legs[leg_idx].leg_to_position(center_pos[0], center_pos[1], center_pos[2], 500);
    }
    delay(500);

    return true;
}

void Hexapod::transform_to_leg_frame(int leg_idx, const double global_dir[2], double local_dir[3]) const
{
    // Convert leg angle from degrees to radians
    double angle_rad = LEG_ANGLES[leg_idx] * PI / 180.0;
    double cos_a = cos(angle_rad);
    double sin_a = sin(angle_rad);

    // Rotate the direction vector by -angle (inverse rotation)
    // This transforms from global frame to leg's local frame
    local_dir[0] = cos_a * global_dir[0] + sin_a * global_dir[1];
    local_dir[1] = -sin_a * global_dir[0] + cos_a * global_dir[1];
    local_dir[2] = 0.0; // Z component is not rotated
}

void Hexapod::generate_curve_point(double t, double start[3], double direction[3],
                                   double step_height, double stride, double result[3]) const
{
    // Swing phase: parabolic arc trajectory
    // X,Y: move forward along direction by 'stride'
    // Z: arc upward (toward 0) by step_height in the middle
    // Since legs work in negative Z space, lifting means adding to Z (making it less negative)

    result[0] = start[0] + direction[0] * stride * t;
    result[1] = start[1] + direction[1] * stride * t;

    // Parabolic height: reaches max at t=0.5
    // Lift upward by adding positive height to negative Z
    // h(t) = 4 * step_height * t * (1 - t)
    result[2] = start[2] + 4.0 * step_height * t * (1.0 - t);
}

void Hexapod::generate_line_point(double t, double start[3], double direction[3],
                                  double stride, double result[3]) const
{
    // Stance phase: straight line on ground
    // Move backward (if stride is negative) to push robot forward

    result[0] = start[0] + direction[0] * stride * t;
    result[1] = start[1] + direction[1] * stride * t;
    result[2] = start[2]; // Stay at same Z height
}