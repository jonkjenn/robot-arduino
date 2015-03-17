#include "Arduino.h"

#ifndef BACHELOR_ENCODER
#define BACHELOR_ENCODER

class Encoder{
    private:
        unsigned char _pinA, _pinB;
        int dir = 0;

        int encAout = 0;
        int encBout = 0;
        int prevAout = 0;
        int prevBout = 0;
        int res[2] = {0,0};

        int stopCount = 0;

        int fDist = 0;
        int bDist = 0;

        int counter = 0;

        void readData();

    public:
        void setup(unsigned char pinA, unsigned char pinB);
        void update();
};

#endif
