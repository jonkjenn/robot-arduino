#include <QTRSensors.h>
#include <Servo.h>
#include <PID_v1.h>

// This example is designed for use with eight QTR-1RC sensors or the eight sensors of a
// QTR-8RC module.  These reflectance sensors should be connected to digital inputs 3 to 10.
// The QTR-8RC's emitter control pin (LEDON) can optionally be connected to digital pin 2, 
// or you can leave it disconnected and change the EMITTER_PIN #define below from 2 to 
// QTR_NO_EMITTER_PIN.

// The setup phase of this example calibrates the sensor for ten seconds and turns on
// the LED built in to the Arduino on pin 13 while calibration is going on.
// During this phase, you should expose each reflectance sensor to the lightest and 
// darkest readings they will encounter.
// For example, if you are making a line follower, you should slide the sensors across the
// line during the calibration phase so that each sensor can get a reading of how dark the
// line is and how light the ground is.  Improper calibration will result in poor readings.
// If you want to skip the calibration phase, you can get the raw sensor readings
// (pulse times from 0 to 2500 us) by calling qtrrc.read(sensorValues) instead of
// qtrrc.readLine(sensorValues).

// The main loop of the example reads the calibrated sensor values and uses them to
// estimate the position of a line.  You can test this by taping a piece of 3/4" black
// electrical tape to a piece of white paper and sliding the sensor across it.  It
// prints the sensor values to the serial monitor as numbers from 0 (maximum reflectance) 
// to 1000 (minimum reflectance) followed by the estimated location of the line as a number
// from 0 to 5000.  1000 means the line is directly under sensor 1, 2000 means directly
// under sensor 2, etc.  0 means the line is directly under sensor 0 or was last seen by
// sensor 0 before being lost.  5000 means the line is directly under sensor 5 or was
// last seen by sensor 5 before being lost.

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {22, 24, 26, 28, 30, 32, 34, 36},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];
Servo ST1,ST2;//ST1 left motor, ST2 right motor

//unsigned int preCalibratedMin[] = {800, 588, 488, 532, 536, 536, 580, 812};
unsigned int preCalibratedMin[] = {2500, 2112, 1830, 1950, 1950, 1850, 1828, 2324};
unsigned int preCalibratedMax = 2500;

double pid_SetPoint,pid_Input,pid_Output;
double aggKp=10, aggKi=0.4, aggKd=1.25;
double consKp=0.1, consKi=0.0, consKd=0.0;
PID myPID(&pid_Input, &pid_Output, &pid_SetPoint,consKp,consKi,consKd,DIRECT);

unsigned int maxPower = 130;
unsigned int stopPower = 90;
unsigned int minPower = 60;//stand still
unsigned int power_range = maxPower - minPower;

//Turn within the limits of maxpower and minpower
//NO Direction 0-255, 0-127 towards right, 129-255 towards left, about.
//Direction -256-255 negativ right, positiv left
void do_turn(int direction)
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

void setup()
{
  delay(500);
  
  ST1.attach(7,1000,2000);
  ST2.attach(8,1000,2000);
  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 1; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  
  for(int i=0;i<NUM_SENSORS;i++)
  {
    qtrrc.calibratedMinimumOn[i] = preCalibratedMin[i];
    qtrrc.calibratedMaximumOn[i] = preCalibratedMax;
  }
  
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(115200);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
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
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
/*  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOff[i]);
    Serial.print(' ');
  }
  Serial.println();*/

  unsigned int position = qtrrc.readLine(sensorValues);
  
  pid_Input = position;
  pid_SetPoint = 3500;
  myPID.SetMode(AUTOMATIC);
  
  Serial.println();

  delay(1000);
  }


void loop()
{
  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);
  unsigned int position = qtrrc.readLine(sensorValues);

  if(position >= 7000 || position ==0)
  {
    ST1.write(stopPower);
    ST2.write(stopPower);
    return;
  }

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  //Serial.println(); // uncomment this line if you are using raw values
  
  Serial.println(position); // comment this line out if you are using raw values
  
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
  if(myPID.Compute())
  {
    do_turn(pid_Output);
      Serial.println("PID output: " + String(pid_Output));
  }
  

    
}

