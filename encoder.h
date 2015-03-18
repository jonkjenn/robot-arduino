#include "Arduino.h"

#ifndef BACHELOR_ENCODER
#define BACHELOR_ENCODER


class Encoder{
    private:
        unsigned char _pinA, _pinB;
        int dir = 0;

        int encAout = 0;
        int encBout = 0;
        int encAoutprev = -1;
        int encBoutprev = -1;
        int prevAout = 0;
        int prevBout = 0;
        int res[2] = {0,0};

        int direction = 0;

        int stopCount = 0;

        double fDist = 0;
        double bDist = 0;

        int counter = 0;

        void readData();

        const int debug = 1;

        const double WHEEL_SIZE = 0.124;

    public:
        void setup(unsigned char pinA, unsigned char pinB);
        void update();
        double getDistance();
};

#endif
