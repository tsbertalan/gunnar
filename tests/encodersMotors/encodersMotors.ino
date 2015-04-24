#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <MemoryFree.h>
#include <gunnar.h>

// Test the motors and encoders.
void setup() {
    pinMode(PIN_ACTIVITYSWITCH, INPUT);
    Serial.begin(9600);           // set up Serial library at 9600 bps
    // Set the speed to start, from 0 (off) to 255 (max speed)
    motor1->run(RELEASE);
    motor2->run(RELEASE);
}


void loop()
{
  loopMinMax(0,1);
  loopMinMax(1,2);
  loopMinMax(0,2);
}


void loopMinMax(int minMtr, int MaxMtr)
{
//     Serial.println(analogRead(2));
//     int spd = constrain(map(analogRead(2), 32, 1016, -255, 255), -255, 255); // 5V
    
//     int spd = constrain(map(analogRead(2), 12, 620, -255, 255), -255, 255); // 3.3V
    int sgns[] = {-1 , 1};
    for(uint8_t isgn=0; isgn<2; isgn++)
    {
        int sgn = sgns[isgn];
        for(int spd=0; abs(spd)<256; spd += sgn)
        {
            updateMtrTest(spd, minMtr, MaxMtr);
            delay(100);
        }
        for(int spd=255*sgn; abs(spd)>1; spd += -sgn)
        {
            updateMtrTest(spd, minMtr, MaxMtr);
            delay(100);
        }
    }
  
}






void stop()
{
  for(uint8_t i=0; i<2; i++)
  {
    bothMtrs[i]->setSpeed(0);
    bothMtrs[i]->run(RELEASE);
  }
}

    






void updateMtrTest(int spd, int minMtr, int maxMtr)
{
  if(digitalRead(PIN_ACTIVITYSWITCH) == HIGH)
  {
    Serial.print(minMtr);
    Serial.print(",");
    Serial.print(maxMtr);
    Serial.print(",");
    Serial.print(spd);
    Serial.print(",");
    Encoder encoders[] = {encoder0, encoder1};
    for(uint8_t i=minMtr; i<maxMtr; i++)
    {
        if(spd < 0)
        {
            bothMtrs[i]->run(BACKWARD);
        }
        else
        {
            bothMtrs[i]->run(FORWARD);
        }
        bothMtrs[i]->setSpeed(abs(spd));
        Serial.print(encoders[i].getSpeed());
        Serial.print(",");
    }
    Serial.println("");
  }
  else
  {
    stop();
  }
}






