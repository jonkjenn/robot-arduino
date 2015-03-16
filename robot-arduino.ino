#include "bachelor-line.h"
#include "Servo.h"

using namespace bachelor;
using namespace std;
LineFollower lf;

void setup()
{
    delay(500);
    Serial.begin(115200);
    Serial.println("Setting up linefollower");
    lf.setup();
    delay(1000);
}


void loop()
{
    lf.update();  
}

