#include <Wire.h>
#include <PID_v1.h>
#include "Adafruit_MotorShield_modified.h"
#include "stats.h"
#include "encoders.h"
#include "pinDefinitions.h"
#include "motorPIDcontrol.h"


// GLOBALS
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor* motor1 = AFMS.getMotor(1);
Adafruit_DCMotor* motor2 = AFMS.getMotor(2);
Adafruit_DCMotor* bothMtrs[] = {motor1, motor2};
Adafruit_DCMotor* leftMtrs[] = {motor1};
Adafruit_DCMotor* riteMtrs[] = {motor2};

Encoder encoder0(encoder0PinA, encoder0PinB, motor1);
Encoder encoder1(encoder1PinA, encoder1PinB, motor2);
Encoder encoders[] = {encoder0, encoder1};


void setup() {
    encodersSetup();
    motorPIDcontrolSetup();
    
    pinMode(PIN_ACTIVITYSWITCH, INPUT);
        
    Serial.begin(115200);
    AFMS.begin(1000);

    // Set the speed to start, from 0 (off) to 255 (max speed)
    motor1->run(RELEASE);
    motor2->run(RELEASE);
}


// How long should we hunt for each speed?
const long waitTime = 8L*1000L; // [ms]
// How large of speeds should we try?
const int maxSpd = 24;
// How much should we increment the setpoint each time?
const int spdInc = maxSpd/2;
// How long should we wait between PID updates?
// Smaller means a faster possible controller response, but possibly also that
// we might miss some encoder ticks.
const long stepDelay = 196L;  // [ms]
int different = -1;
void loop()
{
//     different = -different;
    // Servo our way to several locations.
    int positions[] = {120, 0};//, 0, 200};
    for(uint8_t ipos=0; ipos<2; ipos++)
    {
        int pos = positions[ipos];
        
        // Allow some time to pass for the PIDs to try to work.
        long startTime = millis();
        for(long tstep=0L; tstep<waitTime; tstep+=stepDelay)
        {
            updateMtrTest(pos);
            multDelayMicroseconds(stepDelay*1000L);
            if(millis() - startTime > waitTime)
                break;
        }
    }
}


void multDelayMicroseconds(long us)
{
    const int maxDelay = 16383;
    int extra = us % maxDelay;
    uint8_t ndelays = (us - extra) / maxDelay;
    for(uint8_t i=0; i<ndelays; i++)
    {
        delayMicroseconds(maxDelay);
    }
    delayMicroseconds(extra);
}


void stop()
{
  for(uint8_t i=0; i<2; i++)
  {
    bothMtrs[i]->setSpeed(0);
    bothMtrs[i]->run(RELEASE);
  }
}


void updateMtrTest(int pos)
{
  if(digitalRead(PIN_ACTIVITYSWITCH) == HIGH)
  {
//    Serial.print(pos);
//    Serial.print(",");
    
//     Encoder encoders[] = {encoder0, encoder1};
    controlMotorPositions(pos, different*pos);

//     for(uint8_t i=0; i<2; i++)
//     {
//      Serial.print(encoders[i].getSpeed());
//      Serial.print(",");
//     }
//    Serial.println("");
  }
  else
  {
    stop();
  }
}












