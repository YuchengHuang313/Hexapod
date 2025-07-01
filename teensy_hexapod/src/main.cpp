#include <Arduino.h>
#include "Hexapod.h"
#include "Leg.h" // for the static geometry constants

// single global hexapod instance
Hexapod hexapod;

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("=== Hexapod Mirror‐Move Test ===");

    // Compute a default tip position (for example, straight out in +Y)
    double x0 = 100;
    double z0 = -200;
    double y_target = 100.0; // change to whatever Y you want

    // pack into the 3‐element array your API expects
    double target_pos[3] = {x0, y_target, z0};

    // call your new mirror‐sweep method
    bool ok = hexapod.hexapod_to_position_mirror(target_pos);

    y_target = -100.0; // change to whatever Y you want
    double target_pos1[3] = {x0, y_target, z0};
    ok &= hexapod.hexapod_to_position_mirror(target_pos1);
    if (ok)
    {
        Serial.println("hexapod_to_position_mirror: SUCCESS");
    }
    else
    {
        Serial.println("hexapod_to_position_mirror: FAILURE");
    }

    hexapod.hexapod_unload();
}

void loop()
{
    // nothing to do after the one‐off move
}
