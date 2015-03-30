#ifndef BACHELOR_DRIVE
#define BACHELOR_DRIVE

#include "Servo.h"
#include  "Arduino.h"
#include "encoder.h"
#include "PID_v1.h"

using DriveCompletedCallback = void(*)();

class Drive{
    private:
        unsigned int stopPower = 90;//stand still
        unsigned long _startTime = 0;
        unsigned long _duration = 0;
        unsigned long _distance = 0;
        Encoder encoderRight;
        Encoder encoderLeft;
        uint8_t leftSpeed= 90;
        uint8_t rightSpeed = 90;

        double encoder_pid_SetPoint,encoder_pid_Input,encoder_pid_Output;
        double encoder_consKp=1.0, encoder_consKi=0.0, encoder_consKd=0.0, encoder_pid_weight = 1.0;
        PID *encoderPID;
        enum State{ DRIVING_MANUAL, DRIVING_DURATION, DRIVING_DISTANCE , STOPPED };

        State state = STOPPED;

        DriveCompletedCallback driveCompletedCallback = nullptr;

    public:
        Servo ST1,ST2;//ST1 left motor, ST2 right motor
        void setup(unsigned int ST1_pin, unsigned int ST2_pin, unsigned char encoder_left_a, unsigned char encoder_left_b, unsigned char encoder_right_a, unsigned char encoder_right_b);
        void driveDuration(unsigned int speed, unsigned long duration, DriveCompletedCallback);
        void driveDistance(unsigned int speed, unsigned long distance, DriveCompletedCallback);
        void drive(unsigned int power1, unsigned int power2, unsigned long duration);
        void update();
        void stop();
        uint32_t getDistance();
};
#endif
