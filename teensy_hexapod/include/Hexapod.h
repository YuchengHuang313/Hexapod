// Hexapod.h

#ifndef HEXAPOD_H
#define HEXAPOD_H

#include "Leg.h"

class Hexapod
{
private:
    Leg legs[6];

public:
    static constexpr int TOTAL_MS = 1000;    // ms per p2p sweep
    static constexpr float STEP_SIZE = 1.0f; // mm per step

    Hexapod();
    bool hexapod_to_position(double x, double y, double z);
    bool hexapod_to_position_mirror(double target_pos[3]);
    bool hexapod_read_all_leg_pos(double leg_servo_pos[6][3]);
    void hexapod_unload();
};

#endif // HEXAPOD_H
