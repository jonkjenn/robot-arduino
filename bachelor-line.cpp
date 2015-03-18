#include "bachelor-line.h"

using namespace std;
using namespace bachelor;

void LineFollower::setup(unsigned int ST1_pin, unsigned int ST2_pin, Drive *driver)
{

    _driver = driver;
    Serial.println("Starting linefollower");
    unsigned char sensor_pins[8] =  {22, 24, 26, 28, 30, 32, 34, 36};
    qtrrc = new QTRSensorsRC(sensor_pins,
            NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
    myPID = new PID(&pid_Input, &pid_Output, &pid_SetPoint,consKp,consKi,consKd,DIRECT);
    ST1.attach(ST1_pin,1000,2000);
    ST2.attach(ST2_pin,1000,2000);

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
    for (int i = 0; i < 1; i++)  // make the calibration take about 10 seconds
    {
        qtrrc->calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    }

    for(int i=0;i<NUM_SENSORS;i++)
    {
        qtrrc->calibratedMinimumOn[i] = preCalibratedMin[i];
        qtrrc->calibratedMaximumOn[i] = preCalibratedMax;
    }

    digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

    //print the calibration minimum values measured when emitters were on
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(qtrrc->calibratedMinimumOn[i]);
        Serial.print(' ');
    }
    Serial.println();

    // print the calibration maximum values measured when emitters were on
    /*  for (int i = 0; i < NUM_SENSORS; i++)
        {
        Serial.print(qtrrc.calibratedMinimumOff[i]);
        Serial.print(' ');
        }*/
    Serial.println();
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        Serial.print(qtrrc->calibratedMaximumOn[i]);
        Serial.print(' ');
    }
    Serial.println();
    /*  for (int i = 0; i < NUM_SENSORS; i++)
        {
        Serial.print(qtrrc.calibratedMaximumOff[i]);
        Serial.print(' ');
        }
        Serial.println();*/

    /*unsigned int position = qtrrc->readLine(sensorValues);

    pid_Input = position;*/
    pid_SetPoint = 3500;
    myPID->SetMode(AUTOMATIC);

    Serial.println();
}

//Turn within the limits of maxpower and minpower
//NO Direction 0-255, 0-127 towards right, 129-255 towards left, about.
//Direction -256-255 negativ right, positiv left
void LineFollower::do_turn(int direction)
{
    double nDirection = (double)(direction)/256.0;
    Serial.println("nDirection: " + String(nDirection));

    if(direction < 0)
    {
        //unsigned int power1 = minPower - nDirection*power_range;
        unsigned int power2 = maxPower + nDirection*power_range;
        Serial.println("Turn right P1: " + String(maxPower) + " P2: " + String(power2)); 
        ST1.write(maxPower);
        ST2.write(power2);
    }
    else
    {
        //    unsigned int power2 = minPower + nDirection*power_range;
        unsigned int power1 = maxPower - nDirection*power_range;
        Serial.println("Turn left P1: " + String(power1) + " P2: " + String(maxPower)); 
        ST2.write(maxPower);
        ST1.write(power1);
    }
}

void LineFollower::update()
{
    //read calibrated sensor values and obtain a measure of the line position from 0 to 5000
    // To get raw sensor values, call:
    //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);
    if(result_ready<0)
    {
        qtrrc->readLine(sensorValues, QTR_EMITTERS_ON,0, &position, &result_ready);
        return;
    }
    else if(result_ready == 0)
    {
        return;
    }

    result_ready = -1;//Next round we will restart readline process

    //unsigned int position = 1;
    //qtrrc->read(sensorValues);
    if(debug){Serial.println("Position: " + String(position));}

    if(previous_position<0){previous_position = position; return;}

    if(position >= 7000 || position ==0)
    {
        ST1.write(stopPower);
        ST2.write(stopPower);
        return;
    }

    double distance = _driver->getDistance();

    double angle = atan2((position-previous_position),distance);

    if(debug){Serial.println("prevpos: " + String(previous_position));}
    if(debug){Serial.println(" pos: " + String(position));}
    if(debug){Serial.println("distance: " + String(distance));}
    if(debug){Serial.println("Angle: " + String(angle));}

    // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
    // 1000 means minimum reflectance, followed by the line position
    if(debug)
    {
        for (unsigned char i = 0; i < NUM_SENSORS; i++)
        {
            Serial.print(sensorValues[i]);
            Serial.print('\t');
        }
        //Serial.println(); // uncomment this line if you are using raw values

        Serial.println(position); // comment this line out if you are using raw values
    }

    pid_Input = position;
    //  double gap =abs(pid_SetPoint-pid_Input);

    /*  if(gap<2500)
        {
        myPID.SetTunings(consKp,consKi,consKd);
        }
        else
        {
        myPID.SetTunings(aggKp,aggKi,aggKd);
        }*/
    if(myPID->Compute())
    {
        //do_turn(pid_Output);
        if(debug){Serial.println("PID output: " + String(pid_Output));}
    }

    previous_position = position;
}
