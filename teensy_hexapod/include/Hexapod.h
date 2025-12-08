// Hexapod.h

#ifndef HEXAPOD_H
#define HEXAPOD_H

#include "Leg.h"
#include "WorkspaceLookup.h"

class Hexapod
{
private:
    Leg legs[6];

public:
    // Type definition for time scaling function
    typedef double (Hexapod::*TimeScalingFunc)(double) const;

    static constexpr int TOTAL_MS = 1000;    // ms per p2p sweep
    static constexpr float STEP_SIZE = 1.0f; // mm per step

    Hexapod();
    bool hexapod_to_position(double x, double y, double z);
    bool hexapod_to_position_mirror(double target_pos[3],
                                    TimeScalingFunc scaling_func = &Hexapod::time_scaling_quad);
    bool hexapod_read_all_leg_pos(double leg_servo_pos[6][3]);
    void hexapod_unload();

    // Tripod gait movement
    // direction: [x, y, z] where:
    //   x,y define movement direction (normalized automatically)
    //   z is ground contact height (negative value, e.g., -150mm)
    // step_height: how high (in mm) to lift foot during swing phase (positive value)
    // Stride length is automatically maximized based on workspace constraints at ground Z
    bool tripod_move(double direction[3], double step_height,
                     TimeScalingFunc scaling_func = &Hexapod::time_scaling_quad);

private:
    // Leg rotation angles (degrees): Leg1=0, Leg2=60, Leg3=120, Leg4=180, Leg5=240, Leg6=300
    static constexpr double LEG_ANGLES[6] = {0.0, 60.0, 120.0, 180.0, 240.0, 300.0};

    // Transform direction from global frame to leg's local frame
    void transform_to_leg_frame(int leg_idx, const double global_dir[2], double local_dir[3]) const;

    // Time scaling functions: returns value from 0 to 1
    // t: normalized progress (0 to 1)
    // returns: scaled progress (0 to 1)
    double time_scaling_quad(double t) const;
    double time_scaling_trapezoidal(double t) const;

    // Tripod gait helper functions
    // Generate curve trajectory point (swing phase - leg in air)
    void generate_curve_point(double t, double start[3], double direction[3],
                              double step_height, double stride, double result[3]) const;

    // Generate line trajectory point (stance phase - leg pushing on ground)
    void generate_line_point(double t, double start[3], double direction[3],
                             double stride, double result[3]) const;
};

#endif // HEXAPOD_H
