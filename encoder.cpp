#include "encoder.h"

void Encoder::setup(unsigned char pinA, unsigned char pinB) {
    _pinA = pinA;
    _pinB = pinB;
    pinMode(_pinA,INPUT);
    pinMode(_pinB,INPUT);
    readData();
    readData();
}

//Backward 0 -> 1, 1-> 3, 3->2, 2->0
//0011 - 3
//0111 - 7
//1110 - 14
//1000 - 8
//Forward 0 -> 2, 2->3, 3->1, 1-> 0
//0010 - 2
//1011 - 11
//1101 - 13
//0100 - 4
void Encoder::update() {

    encAout = digitalRead(_pinA);
    encBout = digitalRead(_pinB);

    if(encAoutprev >= 0 && encBoutprev >= 0){
        if(encAout == encAoutprev && encBout != encBoutprev && encBout == encAout)
        {
            direction = 1;
            fDist++;
            //Serial.println("Forward");
        }
        else if(encBout == encBoutprev && encAout != encAoutprev && encAout == encBout)
        {
            direction = -1;
            bDist++;
            //Serial.println("Backward");
        }
        else
        {
            //Serial.println("Not moving");
        }
    }

    encAoutprev = encAout;
    encBoutprev = encBout;

    counter++;
    //Serial.print("12: " + String(d12) + " 13: " + String(d13) + "\n");

    if(debug && counter%500==0)
    {  //Serial.print("Direction : " + String(dir) + " Val: " +String((res[0] <<2)|res[1]) + " A: " + String(encAout) + " B: " + String(encBout)+ "\n");
        Serial.print("Distance forward: " + String(fDist/64.0/18.75 * PI * 0.124) + " backward: " + String(bDist/64.0/18.75 * PI * 0.124) + "\n");
    }
}

double Encoder::getDistance()
{
    double val = 0;
    if(fDist>bDist)
    {
        val = fDist/64.0/18.75 * PI * WHEEL_SIZE;
    }
    else
    {
        val = -bDist/64.0/18.75 * PI * WHEEL_SIZE;
    }

    if(debug && counter%500==0){Serial.println("Encoder val: " + String(val));}


    return val;
}

void Encoder::readData()
{
    res[0] = res[1]; 

    encAout = digitalRead(_pinA);
    encBout = digitalRead(_pinB);

    if(encAoutprev > 0 && encBoutprev > 0){
        if(encAout == encAoutprev && encBout != encBoutprev && encBout == encAout)
        {
            direction = 1;
        }
        else if(encBout == encBoutprev && encAout != encAoutprev && encAout == encBout)
        {
            direction = -1;
        }
    }

    res[1] = (encAout << 1) | encBout;
}
