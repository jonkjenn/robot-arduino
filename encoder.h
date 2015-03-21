#include "Arduino.h"

#ifndef BACHELOR_ENCODER
#define BACHELOR_ENCODER


class Encoder{
    private:
        unsigned char _pinA, _pinB;
        int dir = 0;

        char encAout = 0;
        char encBout = 0;
        char encAoutprev = -1;
        char encBoutprev = -1;
        char prevAout = 0;
        char prevBout = 0;

        char direction = 0;

        float fDist = 0;
        float bDist = 0;

        int counter = 0;

        void readData();

        const char debug = 0;

        const float WHEEL_SIZE = 0.124;

    public:
        void setup(unsigned char pinA, unsigned char pinB);
        void update();
        float getDistance();
};

#endif
