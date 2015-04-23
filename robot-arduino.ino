#include "Servo.h"
#include "arduinocomm.h"
#include "QTRSensors.h"

using namespace std;
Arduinocomm *serial;

#define PWM_PIN_1 6
#define PWM_PIN_2 5

Servo ST1,ST2;//ST1 left motor, ST2 right motor
QTRSensors q{2500};

uint16_t preCalibratedMin[8]  = {970, 760, 620, 680, 680, 680, 680, 950};
uint16_t preCalibratedMax[8]  = {2500,2500,2500,2500,2500,2500,2500,2500};
uint16_t position = 0;
int result_ready = -1;
uint16_t sensorValues[8];

void parsepacket()
{
    uint8_t s = serial->packet_size;
    if(s>0)
    {
        //serial->sendcustombyte(serial->packet_buffer[0]);
        switch(serial->packet_buffer[0])
        {
            case Arduinocomm::DRIVE:
                ST1.write(serial->packet_buffer[1]);
                ST2.write(serial->packet_buffer[2]);
                break;
        }
    }
    serial->packet_ready = false;
}

void setup()
{
    init();
    delay(500);

    ST1.attach(PWM_PIN_1,1000,2000);
    ST2.attach(PWM_PIN_2,1000,2000);

    serial = new Arduinocomm();

    Serial.begin(115200);
    delay(100);
}

void loop()
{
    if(result_ready < 0)
    {
        q.readLine(sensorValues, QTR_EMITTERS_ON,0, &position, &result_ready, preCalibratedMin, preCalibratedMax);
    }
    else if(result_ready == 0)
    {
        q.update();
    }
    else
    {
        //Serial.println("Position:");
        //Serial.println(position);
        result_ready = -1;
    }

    /*serial->update();
    if(serial->packet_ready)
    {
        parsepacket();
    }*/
}
