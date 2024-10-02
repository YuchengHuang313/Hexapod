// Hexapod.h

#ifndef HEXAPOD_H
#define HEXAPOD_H

#include "Leg.h"

class Hexapod {
private:
    Leg legs[6];
    

public:
    Hexapod();
    int hexapod_to_position(float x, float y, float z);
    int check_status();
};

#endif // HEXAPOD_H
