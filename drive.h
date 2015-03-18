#ifndef BACHELOR_DRIVE
#define BACHELOR_DRIVE

#include "Servo.h"
#include  "Arduino.h"
#include "encoder.h"

class Drive{
    private:
        Servo ST1,ST2;//ST1 left motor, ST2 right motor
        unsigned int stopPower = 90;//stand still
        unsigned long _startTime = 0;
        unsigned int _duration = 1000;
        Encoder encoderRight;
        Encoder encoderLeft;

    public:
        void setup(unsigned int ST1_pin, unsigned int ST2_pin, unsigned char encoder_left_a, unsigned char encoder_left_b, unsigned char encoder_right_a, unsigned char encoder_right_b);
        void driveForward(unsigned int speed, unsigned int duration);
        void update();
        double getDistance();
};
#endif
