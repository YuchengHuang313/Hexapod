#include <Arduino.h>
#include "Hexapod.h"
#include "Leg.h" // for the static geometry constants

// single global hexapod instance
Hexapod hexapod;

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("=== Hexapod Tripod Gait Test ===");

    // Define working height (ground contact Z)
    double ground_z = -165.0;  // Legs will contact ground at z=-165mm
    double step_height = 50.0; // Lift 20mm during swing phase

    // Walk in +Y direction multiple times
    Serial.println("Starting tripod gait - walking in +Y direction...");
    double forward_dir[3] = {0.0, 1.0, ground_z}; // Y=1 (positive Y direction), Z=ground contact

    int num_steps = 5; // Number of times to repeat the walking motion
    bool ok = true;

    for (int i = 0; i < num_steps; i++)
    {
        Serial.printf("Step %d/%d\n", i + 1, num_steps);
        ok = hexapod.tripod_move(forward_dir, step_height);

        if (!ok)
        {
            Serial.printf("Tripod move failed at step %d\n", i + 1);
            break;
        }
    }

    if (ok)
    {
        Serial.println("Walking complete: SUCCESS");
    }
    else
    {
        Serial.println("Walking complete: FAILURE");
    }

    Serial.println("=== Test Complete ===");
    hexapod.hexapod_unload();
}

void loop()
{
    // nothing to do after the test
}
