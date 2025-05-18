#include "LobotSerialServoControl.h"
#include "Hexapod.h"
#include "Kinematics.h"
#include <Arduino.h>
#include <math.h>

void setup()
{
    Serial.begin(115200);
    delay(1000);

    // 1) instantiate the leg
    Leg leg1(1, 1, 2, 3);
    delay(1000);

    // 2) neutral X/Z from your class constants
    float x0 = Leg::HIP_TO_KNEE + Leg::ANKLE_TO_TIP;
    float z0 = Leg::KNEE_TO_ANKLE;

    // ---- Forward sweep (+150 → -150) ----
    float y_start = +150.0f, y_end = -150.0f;
    {
        // Compute straight‐line distance & steps
        float dy = y_end - y_start;
        float dist = fabsf(dy);
        int steps = int(dist / Leg::STEP_SIZE + 0.5f);
        if (steps < 1)
            steps = 1;

        // Compute per-step time and pause
        int ms_move = Leg::TOTAL_MS / steps;
        int pause = (ms_move / 4 >= 1 ? ms_move / 4 : 1);

        // Estimate total time: (steps+2)*(move + pause)
        int est_ms = (steps + 2) * (ms_move + pause);
        Serial.printf("\nForward sweep: %d steps, ~%dms total\n", steps, est_ms);

        // Run the sweep
        leg1.leg_p2p(x0, y_start, z0,
                     x0, y_end, z0);
        leg1.leg_read_position();
    }

    delay(500);

    // ---- Backward sweep (-150 → +150) ----
    y_start = -150.0f;
    y_end = +150.0f;
    {
        float dy = y_end - y_start;
        float dist = fabsf(dy);
        int steps = int(dist / Leg::STEP_SIZE + 0.5f);
        if (steps < 1)
            steps = 1;

        int ms_move = Leg::TOTAL_MS / steps;
        int pause = (ms_move / 4 >= 1 ? ms_move / 4 : 1);

        int est_ms = (steps + 2) * (ms_move + pause);
        Serial.printf("\nBackward sweep: %d steps, ~%dms total\n", steps, est_ms);

        leg1.leg_p2p(x0, y_start, z0,
                     x0, y_end, z0);
        leg1.leg_read_position();
    }

    // 3) unload and finish
    Serial.println("\nUnloading servos...");
    leg1.leg_unload_all();
    Serial.println("Test complete.");

    while (true)
        delay(1000);
}

void loop()
{
    // none
}
