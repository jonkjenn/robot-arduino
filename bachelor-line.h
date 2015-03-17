#include "QTRSensors.h"
#include "PID_v1.h"
#include "Arduino.h"
#include "Servo.h"
#include <stdio.h>

namespace bachelor{

class LineFollower{

    private:
    // This example is designed for use with eight QTR-1RC sensors or the eight sensors of a
    // QTR-8RC module.  These reflectance sensors should be connected to digital inputs 3 to 10.
    // The QTR-8RC's emitter control pin (LEDON) can optionally be connected to digital pin 2, 
    // or you can leave it disconnected and change the EMITTER_PIN #define below from 2 to 
    // QTR_NO_EMITTER_PIN.

    // The setup phase of this example calibrates the sensor for ten seconds and turns on
    // the LED built in to the Arduino on pin 13 while calibration is going on.
    // During this phase, you should expose each reflectance sensor to the lightest and 
    // darkest readings they will encounter.
    // For example, if you are making a line follower, you should slide the sensors across the
    // line during the calibration phase so that each sensor can get a reading of how dark the
    // line is and how light the ground is.  Improper calibration will result in poor readings.
    // If you want to skip the calibration phase, you can get the raw sensor readings
    // (pulse times from 0 to 2500 us) by calling qtrrc.read(sensorValues) instead of
    // qtrrc.readLine(sensorValues).

    // The main loop of the example reads the calibrated sensor values and uses them to
    // estimate the position of a line.  You can test this by taping a piece of 3/4" black
    // electrical tape to a piece of white paper and sliding the sensor across it.  It
    // prints the sensor values to the serial monitor as numbers from 0 (maximum reflectance) 
    // to 1000 (minimum reflectance) followed by the estimated location of the line as a number
    // from 0 to 5000.  1000 means the line is directly under sensor 1, 2000 means directly
    // under sensor 2, etc.  0 means the line is directly under sensor 0 or was last seen by
    // sensor 0 before being lost.  5000 means the line is directly under sensor 5 or was
    // last seen by sensor 5 before being lost.

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

        // sensors 0 through 7 are connected to digital pins 3 through 10, respectively
        QTRSensorsRC *qtrrc;
        unsigned int sensorValues[NUM_SENSORS];
        Servo ST1,ST2;//ST1 left motor, ST2 right motor

        //unsigned int preCalibratedMin[] = {800, 588, 488, 532, 536, 536, 580, 812};
        unsigned int preCalibratedMin[8] = {2500, 2112, 1830, 1950, 1950, 1850, 1828, 2324};
        unsigned int preCalibratedMax = 2500;

        double pid_SetPoint,pid_Input,pid_Output;
        double aggKp=10, aggKi=0.4, aggKd=1.25;
        double consKp=0.1, consKi=0.0, consKd=0.0;
        PID *myPID;

        unsigned int maxPower = 120;
        unsigned int stopPower = 90;//stand still
        unsigned int minPower = 70;
        unsigned int power_range = maxPower - minPower;

        void do_turn(int direction);


        public:
            void update();
            void setup(unsigned int ST1_pin, unsigned int ST2_pin);
};

}
