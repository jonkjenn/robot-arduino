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

#define ENCODER_LEFT_A 42
#define ENCODER_LEFT_B 43
#define ENCODER_RIGHT_A 40
#define ENCODER_RIGHT_B 41

void setup()
{
    delay(500);
    Serial.begin(115200);
    Serial.println("Setting up linefollower");
    //lf.setup(3,4);
    driver.setup(ST1_pin, ST2_pin, ENCODER_LEFT_A, ENCODER_LEFT_B, ENCODER_RIGHT_A, ENCODEr_RIGHT_B);
    driver.driveForward(120,1000);
}


void loop()
{
    driver.update();
    //lf.update();  
}

