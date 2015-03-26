#include "bachelor-line.h"
#include "drive.h"
#include "Servo.h"
#include "encoder.h"
#include "arduinocomm.h"

using namespace bachelor;
using namespace std;
LineFollower lf;
Drive driver;
Arduinocomm *serial;

#define ST1_pin 3
#define ST2_pin 4

#define ENCODER_LEFT_A 0
#define ENCODER_LEFT_B 1
#define ENCODER_RIGHT_A 3
#define ENCODER_RIGHT_B 2

unsigned long s2 = 0;
unsigned long s1 = 0;

void parsepacket()
{
    uint8_t s = serial->packet_size;
    if(s>0)
    {
        switch(serial->packet_buffer[0])
        {
            case Arduinocomm::DRIVE_DURATION:
                //serial->sendcustombyte(55);
                serial->writeok();
                //serial->sendcustombyte(serial->packet_buffer[1]);
                //serial->sendcustombyte(serial->packet_buffer[2]);
                /*serial->sendcustombyte(serial->packet_buffer[3]);
                serial->sendcustombyte(serial->packet_buffer[4]);
                serial->sendcustombyte(serial->packet_buffer[5]);
                serial->sendcustombyte(serial->packet_buffer[3]);
                serial->sendcustombyte(serial->packet_buffer[4]);
                serial->sendcustombyte(serial->packet_buffer[5]);*/
                driver.driveForward(serial->packet_buffer[1],serial->read_uint32(serial->packet_buffer,2));               
            break;
        }
    }
    serial->packet_ready = false;
}

void setup()
{
    init();
    delay(500);
    //Serial.println("test");

    serial = new Arduinocomm();

    driver.setup(ST1_pin, ST2_pin, ENCODER_LEFT_A, ENCODER_LEFT_B, ENCODER_RIGHT_A, ENCODER_RIGHT_B);
    //lf.setup(&driver);

    //driver.driveForward(110,1000);
    //driver.drive(110,100,3000);
    s2 = micros();
}

unsigned int c = 0;
void loop()
{
    //unsigned long start = micros();
    //s2 = micros();

    driver.update();

    //Serial.println(micros()-s2);
    //if(c%10000 == 0){Serial.println((micros()-s2)/10000);s2 = micros();}
    //Serial.println("Duration 1: " + String(micros()-start));

    //start = micros();
    //s2 = micros();

    //lf.update();  

    serial->update();
    if(serial->packet_ready)
    {
        //serial->sendcustombyte(66);
        parsepacket();
    }

    //serial->sendcustombyte(99);

    //Serial.println(micros()-s2);
//    if(c%10000 == 0){Serial.println((micros()-s2)/10000);s2 = micros();}
    c++;

    //delay(3000);

    //Serial.println("Duration 2: " + String(micros()-start));
}

