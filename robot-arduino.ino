#include "bachelor-line.h"
#include "drive.h"
#include "Servo.h"
#include "encoder.h"

using namespace bachelor;
using namespace std;
LineFollower lf;
Drive driver;

#define ST1_pin 3
#define ST2_pin 4

#define ENCODER_LEFT_A 0
#define ENCODER_LEFT_B 1
#define ENCODER_RIGHT_A 3
#define ENCODER_RIGHT_B 2

unsigned long s2 = 0;
unsigned long s1 = 0;
void setup()
{
    init();
    delay(500);
    Serial.begin(115200);
    Serial.println("Setting up linefollower");
    Serial.println("test");
    driver.setup(ST1_pin, ST2_pin, ENCODER_LEFT_A, ENCODER_LEFT_B, ENCODER_RIGHT_A, ENCODER_RIGHT_B);
    lf.setup(ST1_pin,ST2_pin, &driver);
    //driver.driveForward(130,2000);
    s1 = micros();
}


unsigned int c = 0;
void loop()
{
    //unsigned long start = micros();
    //s2 = micros();
    //driver.update();
    //Serial.println(micros()-s2);
    //if(c%10000 == 0){Serial.println((micros()-s2)/10000);s2 = micros();}
    //Serial.println("Duration 1: " + String(micros()-start));

    //start = micros();
    //s1 = micros();
    lf.update();  
    //Serial.println(micros()-s1);
//    if(c%10000 == 0){Serial.println((micros()-s1)/10000);s1 = micros();}
    //c++;

    //Serial.println("Duration 2: " + String(micros()-start));
}

