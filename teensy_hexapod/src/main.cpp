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
    double x = 140;
    double y = -150; // change to whatever Y you want
    double z = -160;
    
    // pack into the 3‐element array your API expects
    double target_pos[3] = {x, y, z};

    // call your new mirror‐sweep method
    bool ok = hexapod.hexapod_to_position_mirror(target_pos);

    // move the the higher middle position directly
    // double target_pos_mid[3] = {37 + 200 + 63.54, 0, 0}; // lift Z a bit
    // ok &= hexapod.hexapod_to_position_mirror(target_pos_mid);
    double target_pos1[3] = {x, -y, z};
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
