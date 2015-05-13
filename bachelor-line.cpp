#include "bachelor-line.h"

using namespace std;

void LineFollower::setup(Servo *ST1,Servo *ST2)
{

    leftMotor = ST1;
    rightMotor = ST2;
    Serial.println("Starting linefollower");
    outerPID = new PID(&out_pid_Input, &out_pid_Output, &out_pid_SetPoint,out_consKp,out_consKi,out_consKd,DIRECT,-256,255,&out_pid_weight);
    out_pid_SetPoint = 0.0;
    outerPID->SetMode(AUTOMATIC);
    innerPID = new PID(&inn_pid_Input, &inn_pid_Output, &inn_pid_SetPoint,inn_consKp,inn_consKi,inn_consKd,DIRECT,-256,255,&inn_pid_weight);
    inn_pid_SetPoint = 0.0;
    innerPID->SetMode(AUTOMATIC);

    Serial.println("Attached motors");
}

//Turn within the limits of maxpower and minpower50
//NO Direction 0-255, 0-127 towards right, 129-255 towards left, about.
//Direction -256-255 negativ right, positiv left
void LineFollower::do_turn(int direction)
{
    double nDirection = (double)(direction)/256.0;
    //Serial.println("nDirection: " + String(nDirection));

    if(direction < 0)
    {
        //unsigned int power1 = minPower - nDirection*power_range;
        unsigned int power2 = maxPower + nDirection*power_range;
        if(power2<90 && power2 >70){power2 = 70;}
        Serial.println("Right");
        Serial.println(power2);
        //Serial.println("Turn right P1: " + String(maxPower) + " P2: " + String(power2)); 
        leftMotor->write(maxPower);
        rightMotor->write(power2);
    }
    else
    {
        //unsigned int power2 = minPower + nDirection*power_range;
        unsigned int power1 = maxPower - nDirection*power_range;
        if(power1<90 && power1 >70){power1 = 70;}
        Serial.println("Left");
        Serial.println(power1);
        //Serial.println("Turn left P1: " + String(power1) + " P2: " + String(maxPower)); 
        rightMotor->write(maxPower);
        leftMotor->write(power1);
    }
}

int dtest = 0;
void LineFollower::update(uint16_t position)
{
    dtest++;
    //read calibrated sensor values and obtain a measure of the line position from 0 to 5000
    // To get raw sensor values, call:
    //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);

    //unsigned int position = 1;
    //qtrrc->read(sensorValues);
    //if(debug){Serial.println("Position: " + String(position));}
    //
    //Serial.println(position);


    if(!collected_startpos){
        previous_position = position;
        prev_dist_center = position - ir_center; 
        collected_startpos = true; 
        dist_center = position - ir_center;
        delta_dist_center = dist_center - prev_dist_center;
        for(int i=0;i<4;i++)
        {
            prev_delta[i] = delta_dist_center;
        }
        return;
    }

    if(position >= 7000 || position == 0)
    {
        if(stopcount>20){
            Serial.println("Stopped");
            leftMotor->write(stopPower);
            rightMotor->write(stopPower);
            previous_position = position;
            return;
        }
        stopcount++;
        return;
    }

    stopcount = 0;

    if(position == previous_position){return;}

    dist_center = position - ir_center;

    //out_pid_weight = 1.0;
    //out_pid_weight = abs(dist_center)/3500.0 * 0.7 + 0.3;
    //myPID->weight = abs(dist_center)/3500.0;
    //myPID->weight = abs(dist_center)/3500.0;

    delta_dist_center = dist_center - prev_dist_center;

    for(int i=3;i>0;i--)
    {
        prev_delta[i] = prev_delta[i-1];
    }

    prev_delta[0] = delta_dist_center;

    //delta_dist_center = (prev_delta[0] + prev_delta[1] + prev_delta[2] + prev_delta[3])/4; //GetMedian(prev_delta);
    delta_dist_center = dist_center - prev_dist_center;//GetMedian(prev_delta,4);

    //if(abs(dist_center) < 1000 && abs(delta_dist_center) < 500){return;}
    //if(abs(dist_center) < 1000){return;}

    //double angle = atan2((position-previous_position),distance);
    //diff = position>=previous_position?position-previous_position:previous_position-position;

    //double angle = atan2((float)distance, (float)dist_center*ir_modifier);

    if(debug && dtest%1 == 0){

              //  Serial.println("previous_position");
              //Serial.println(previous_position);
              Serial.println("dist_center");
              Serial.println(dist_center);
              Serial.println("delta_dist_center");
              Serial.println(delta_dist_center);
    }

    out_pid_Input = dist_center;


    //  double gap =abs(pid_SetPoint-pid_Input);

    /*  if(gap<2500)
        {
        myPID.SetTunings(consKp,consKi,consKd);
        }
        else
        {
        myPID.SetTunings(aggKp,aggKi,aggKd);
        }*/

    outerPID->Compute();
//    Serial.println("Outer pid");
    //Serial.println(out_pid_Output);
    inn_pid_SetPoint = out_pid_Output;
    inn_pid_Input = delta_dist_center;

    if(innerPID->Compute())
    {
        //Serial.println("inner pid");
        //Serial.println(inn_pid_Output);
        do_turn(inn_pid_Output);
        //if(debug){Serial.println("PID output: " + String(pid_Output));}
    }

    prev_dist_center = dist_center;
    previous_position = position;
}

int16_t GetMedian(int16_t *daArray, int size) {
    // Allocate an array of the same size and sort it.
    int16_t* dpSorted = new int16_t[4];
    for (int i = 0; i < size; ++i) {
        dpSorted[i] = daArray[i];
    }
    for (int i = size - 1; i > 0; --i) {
        for (int j = 0; j < i; ++j) {
            if (dpSorted[j] > dpSorted[j+1]) {
                double dTemp = dpSorted[j];
                dpSorted[j] = dpSorted[j+1];
                dpSorted[j+1] = dTemp;
            }
        }
    }

    // Middle or average of middle values in the sorted array.
    double dMedian = 0.0;
    if ((size % 2) == 0) {
        dMedian = (dpSorted[size/2] + dpSorted[(size/2) - 1])/2.0;
    } else {
        dMedian = dpSorted[size/2];
    }
    delete [] dpSorted;
    return dMedian;
}
