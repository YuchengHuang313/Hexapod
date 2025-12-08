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
    double ground_z = -165.0;  // Legs will contact ground at z=-150mm
    double step_height = 20.0; // Lift 15mm during swing phase

    // Move legs to starting extreme positions in two groups to avoid overcurrent
    // Group 1 (legs 1,3,5) starts at backward extreme, Group 2 (legs 2,4,6) starts at forward extreme
    Serial.println("Moving legs to starting positions...");

    // Calculate the extreme positions based on workspace
    // Using same logic as tripod_move: stride = radius * 2 * 0.75
    double stride_half = 83.0 * 0.75; // Safe estimate: radius ~83mm at z=-165, use 75% safety
    double move_dir[2] = {0.0, 1.0};  // Moving in +Y direction

    double center_x = 168.0;
    double center_y = 0.0;

    // Backward extreme: center - direction * half_stride
    double backward_pos[3] = {
        center_x - move_dir[0] * stride_half,
        center_y - move_dir[1] * stride_half,
        ground_z};

    // Forward extreme: center + direction * half_stride
    double forward_pos[3] = {
        center_x + move_dir[0] * stride_half,
        center_y + move_dir[1] * stride_half,
        ground_z};

    Serial.printf("Backward extreme: [%.1f, %.1f, %.1f]\n", backward_pos[0], backward_pos[1], backward_pos[2]);
    Serial.printf("Forward extreme: [%.1f, %.1f, %.1f]\n", forward_pos[0], forward_pos[1], forward_pos[2]);

    // Move Group 1 (legs 1,3,5) to backward extreme first
    bool ok = hexapod.hexapod_to_position_mirror(backward_pos);
    if (!ok)
    {
        Serial.println("Failed to position Group 1 to backward extreme!");
        hexapod.hexapod_unload();
        return;
    }
    delay(500); // Let servos settle

    Serial.println("Group 1 (legs 1,3,5) at backward extreme - ready!");

    // Note: We don't move Group 2 separately since hexapod_to_position_mirror moves all legs
    // The first phase of tripod_move will position them correctly

    // Walk in +Y direction multiple times
    Serial.println("Starting tripod gait - walking in +Y direction...");
    double forward_dir[3] = {0.0, 1.0, ground_z}; // Y=1 (positive Y direction), Z=ground contact

    int num_steps = 5; // Number of times to repeat the walking motion
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
