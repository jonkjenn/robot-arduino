#include "Servo.h"
#include "arduinocomm.h"
#include "QTRSensors.h"

using namespace std;
Arduinocomm *serial;

#define PWM_PIN_1 6
#define PWM_PIN_2 5

const bool DEBUG_QTR_SENSORS = 0;
const bool DEBUG_SERIAL = 0;
const bool DEBUG_ROBOT_ARDUION = 0;

Servo ST1,ST2;//ST1 left motor, ST2 right motor
unsigned int max_sensor_timeout_ms = 1200;
QTRSensors q{max_sensor_timeout_ms};

int debug = DEBUG_ROBOT_ARDUION;

uint16_t preCalibratedMin[8]  ={750 ,710,515 ,660 ,616 ,660 ,570, 820}; //{936, 756, 584, 628, 668, 668, 668, 950};//{936, 756, 584, 628, 668, 668, 668, 950};
uint16_t preCalibratedMax[8]  = {max_sensor_timeout_ms,max_sensor_timeout_ms,max_sensor_timeout_ms,max_sensor_timeout_ms,max_sensor_timeout_ms,max_sensor_timeout_ms,max_sensor_timeout_ms,max_sensor_timeout_ms};
uint16_t position = 0;
int result_ready = -1;
uint16_t sensorValues[8];

int count = 0;

void parsepacket()
{
    uint8_t s = serial->packet_size;
    if(s>0)
    {
        //serial->sendcustombyte(serial->packet_buffer[0]);
        switch(serial->packet_buffer[0])
        {
            case Arduinocomm::DRIVE:
                uint8_t l = serial->packet_buffer[1];
                uint8_t r = serial->packet_buffer[2];
                if(l == 90 && r == 90)
                {
                    serial->write_ok();
                }
                if(!debug)
                {
                    ST1.write(serial->packet_buffer[1]);
                    ST2.write(serial->packet_buffer[2]);
                }
                break;
        }
    }
    serial->packet_ready = false;
}

void setup()
{
    init();
    delay(500);

    if(!debug)
    {
        ST1.attach(PWM_PIN_1,1000,2000);
        ST2.attach(PWM_PIN_2,1000,2000);
        ST1.write(90);
        ST1.write(90);
    }

    serial = new Arduinocomm();
    serial->debug = DEBUG_SERIAL;
    q.debug_output = DEBUG_QTR_SENSORS;

    Serial.begin(115200);
    delay(100);
}

void loop()
{
    count++;
    if(result_ready < 0)
      {
          q.readLine(sensorValues, QTR_EMITTERS_ON,0, &position, &result_ready, preCalibratedMin, preCalibratedMax);
      }
      else if(result_ready == 0)
      {
          //q.update();
      }
      else
      {
        if(debug)
        {
            Serial.println("Position:");
            Serial.println(position);
        }

        serial->write_line_position(position);
        result_ready = -1;
    }

    if(!debug)
    {
        serial->update();
        if(serial->packet_ready)
        {
            parsepacket();
        }
    }
}
