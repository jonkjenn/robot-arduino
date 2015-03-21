#include "drive.h"

void Drive::setup(unsigned int ST1_pin, unsigned int ST2_pin, unsigned char encoder_left_a, unsigned char encoder_left_b, unsigned char encoder_right_a, unsigned char encoder_right_b)
{
    ST1.attach(ST1_pin,1000,2000);
    ST2.attach(ST2_pin,1000,2000);

    encoderRight.setup(encoder_right_a,encoder_right_b);
    encoderLeft.setup(encoder_left_a,encoder_left_b);
}

void Drive::driveForward(unsigned int speed, unsigned int duration)
{
    Serial.println("Driving with speed: " + String(speed));
    _duration = duration;
    _startTime = micros();
    ST1.write(speed);
    ST2.write(speed);
}

void Drive::update()
{
    encoderLeft.update();
    encoderRight.update();
    if(_duration > 0 && ((micros()-_startTime) > _duration*(unsigned long)1000))
    {
        Serial.println("Stopping");
        ST1.write(stopPower);
        ST2.write(stopPower);
        _duration = 0;
    }
}

double Drive::getDistance()
{
    return (encoderRight.getDistance() + encoderLeft.getDistance())/2;
}
