#include <Arduino.h>
#include "Leg.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== Leg Movement Test Suite (Extended with Start Moves) ===");

    // Instantiate leg
    Leg leg1(1, 1, 2, 3);
    delay(1000);

    // Compute neutral X/Z
    float x0 = Leg::HIP_TO_KNEE + Leg::ANKLE_TO_TIP;
    float z0 = Leg::KNEE_TO_ANKLE;

    // 1) Extended forward/backward sweep (±150 mm)
    Serial.println("\n1) Extended Forward/backward sweep ±150mm");
    // Move to start position
    leg1.leg_to_position(x0, 150.0f, z0, Leg::TOTAL_MS);
    leg1.leg_read_position();
    delay(500);
    // Sweep forward to -150
    leg1.leg_p2p(x0, 150.0f, z0, x0, -150.0f, z0);
    leg1.leg_read_position();
    delay(500);
    // Move back to start
    leg1.leg_to_position(x0, -150.0f, z0, Leg::TOTAL_MS);
    leg1.leg_read_position();
    delay(500);

    // 2) High lift up and deep drop down (±100 mm)
    Serial.println("\n2) High lift (z+=100) and deep drop (z-=100)");
    // Move to neutral start
    leg1.leg_to_position(x0, 0.0f, z0, Leg::TOTAL_MS);
    leg1.leg_read_position();
    delay(500);
    // Lift up
    leg1.leg_p2p(x0, 0.0f, z0, x0, 0.0f, z0 + 100.0f);
    leg1.leg_read_position();
    delay(500);
    // Drop down
    leg1.leg_p2p(x0, 0.0f, z0 + 100.0f, x0, 0.0f, z0 - 100.0f);
    leg1.leg_read_position();
    delay(500);

    // 3) Larger diagonal step forward-right (50mm x, 100mm y, -50mm z)
    Serial.println("\n3) Larger diagonal step forward-right");
    // Move to diagonal start
    leg1.leg_to_position(x0, 0.0f, z0, Leg::TOTAL_MS);
    leg1.leg_read_position();
    delay(500);
    // Step forward-right
    leg1.leg_p2p(x0, 0.0f, z0, x0 + 50.0f, 100.0f, z0 - 50.0f);
    leg1.leg_read_position();
    delay(500);
    // Return to start
    leg1.leg_p2p(x0 + 50.0f, 100.0f, z0 - 50.0f, x0, 0.0f, z0);
    leg1.leg_read_position();
    delay(500);

    // 4) Large circle around neutral (radius 50mm Y, 25mm Z)
    Serial.println("\n4) Large circle around neutral");
    // Move to circle start
    leg1.leg_to_position(x0, 50.0f, z0, Leg::TOTAL_MS);
    leg1.leg_read_position();
    delay(500);
    const int CIRCLE_STEPS = 16;
    const float RY = 50.0f; // 50 mm radius in Y
    const float RZ = 25.0f; // 25 mm radius in Z
    for (int i = 0; i <= CIRCLE_STEPS; ++i)
    {
        float angle = TWO_PI * i / CIRCLE_STEPS;
        float y = RY * cos(angle);
        float z = z0 + RZ * sin(angle);
        leg1.leg_to_position(x0, y, z, Leg::TOTAL_MS / CIRCLE_STEPS);
        delay(Leg::TOTAL_MS / CIRCLE_STEPS / 2);
    }
    leg1.leg_read_position();

    // Unload
    Serial.println("\nUnloading servos...");
    leg1.leg_unload_all();
    Serial.println("=== Extended Test Suite Complete ===");
}

void loop()
{
    // No further actions
}