#include "drive.h"

void Drive::setup(unsigned int ST1_pin, unsigned int ST2_pin, unsigned char encoder_left_a, unsigned char encoder_left_b, unsigned char encoder_right_a, unsigned char encoder_right_b)
{
    ST1.attach(ST1_pin,1000,2000);
    ST2.attach(ST2_pin,1000,2000);

    encoderRight.setup(encoder_right_a,encoder_right_b);
    encoderLeft.setup(encoder_left_a,encoder_left_b);

    distancePID = new PID(&encoder_pid_Input, &encoder_pid_Output, &encoder_pid_SetPoint,encoder_consKp,encoder_consKi,encoder_consKd,DIRECT,-10,10,&encoder_pid_weight);
    encoder_pid_SetPoint = 0.0;
    distancePID->SetMode(AUTOMATIC);
}

//duration in milliseconds
void Drive::driveDuration(unsigned int speed, unsigned long duration, DriveCompletedCallback callback)
{
    if(state != STOPPED){return;}

    driveCompletedCallback = callback;

    state = DRIVING_DURATION;
    encoderRight.reset();
    encoderLeft.reset();

    //Serial.println("Driving with speed: " + String(speed));
    _duration = duration * 1000;
    _startTime = micros();
    ST1.write(speed);
    ST2.write(speed);

    leftSpeed = speed;
    rightSpeed = speed;
}

void Drive::driveDistance(unsigned int speed, unsigned long distance, DriveCompletedCallback callback)
{
    if(state != STOPPED){return;}

    driveCompletedCallback = callback;

    encoderRight.reset();
    encoderLeft.reset();

    state = DRIVING_DISTANCE;
    _distance = distance*1000;
    ST1.write(speed);
    ST2.write(speed);

    leftSpeed = speed;
    rightSpeed = speed;
}

void Drive::drive(unsigned int power1, unsigned int power2, unsigned long duration)
{
    if(state == DRIVING_MANUAL || state == STOPPED){
        state = DRIVING_MANUAL;
        _duration = duration * 1000;
        _startTime = micros();
        ST1.write(power1);
        ST2.write(power2);
    }
}

void Drive::update()
{
    encoderLeft.update();
    encoderRight.update();

    if(state == DRIVING_DURATION)
    {
        if(_duration > 0 && ((micros()-_startTime) > _duration))
        {
            //Serial.println("Stopping");
            stop();
            _duration = 0;

            if(driveCompletedCallback != nullptr) { driveCompletedCallback();}
        }
    }else if(state == DRIVING_DISTANCE)
    {
        if(encoderLeft.getDistance() >= _distance)
        {
            stop();
            _distance = 0;

            if(driveCompletedCallback != nullptr) { driveCompletedCallback();}
        }
        else
        {
            encoder_pid_Input = encoderRight.getSpeed() - encoderLeft.getSpeed();
            encoderPID->Compute();
            ST2.write(rightSpeed + encoder_pid_Output);
        }
    }
}

void Drive::stop()
{
    encoderRight.reset();
    encoderLeft.reset();

    ST1.write(stopPower);
    ST2.write(stopPower);
    state = STOPPED;
}

uint32_t Drive::getDistance()
{
    return (encoderRight.getDistance() + encoderLeft.getDistance());
}
