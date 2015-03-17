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

    readData();  
   
    //Serial.print("Res0: " + String(res[0]) + " Res1: " + String(res[1]) + "\n");
    
    if(res[0] == res[1]){
      if(stopCount > 50){dir=0;stopCount = 0;}else{stopCount++;}
    }
    else
    {
      switch((res[0] << 2) | res[1])
      {
        //backward
        case 3:
        case 7:
        case 14:
        case 8:
          dir = -1;
          bDist++;
          break;
        //forward
        case 2:
        case 11:
        case 13:
        case 4:
          dir = 1;
          fDist++;
          break;
      }
   }
   counter++;
//    Serial.print("12: " + String(d12) + " 13: " + String(d13) + "\n");

  if(counter%10000==0)
  {  //Serial.print("Direction : " + String(dir) + " Val: " +String((res[0] <<2)|res[1]) + " A: " + String(encAout) + " B: " + String(encBout)+ "\n");
    Serial.print("Distance forward: " + String(fDist/64/18.75 * PI * 0.124) + " backward: " + String(bDist/64/18.75 * PI * 0.124) + "\n");
  }
}

void Encoder::readData()
{
   res[0] = res[1];  

   encAout = digitalRead(_pinA);
   encBout = digitalRead(_pinB);
    
   res[1] = (encAout << 1) | encBout;
}
