#ifndef VISION_H
#define VISION_H
#include <Servo.h>
#include "constants.h"
#include "utils.h"
#include "Arduino.h"




const int PANOFFSET = 10; // Larger is further to the left.
const int TILTOFFSET = 8; // Larger is further down.
const int NEUTRALPAN = -PANOFFSET;
const int NEUTRALTILT = -TILTOFFSET;

Servo *servoTilt;
Servo *servoPan;

class Sensors
{
public:
    void init()
    {
        pinMode(SONARPIN, INPUT);
        
        servoTilt->attach(TILTSERVOPIN);
        servoPan->attach(PANSERVOPIN);
        
        setPan(0);
        setTilt(0);
    }

    void setPan(int pos)
    {
        // -90 <= pos < 90
        if(!servoPan->attached())
        {
            servoPan->attach(PANSERVOPIN);
        }
        servoPan->write(pos+90+PANOFFSET);
    }

    void setTilt(int pos)
    {
        // -90 <= pos < 90
        if(!servoTilt->attached())
        {
            servoTilt->attach(TILTSERVOPIN);
        }
        servoTilt->write(-pos+90+TILTOFFSET);
    }

    void disablePan()
    {
        setPan(0);
        if(servoPan->attached())
            servoPan->detach();
    }

    void disableTilt()
    {
        setTilt(0);
        if(servoTilt->attached())
            servoTilt->detach();
    }

    void disableServos()
    {
        disablePan();
        disableTilt();
    }

    float getSonarDist()
    {
        return getSonarDist(4);
    }


    float getSonarDist(int nSonarReadings)
    {
    //     Serial.print("measuring distance ... ");
        float dist = 0;
        for(uint8_t i=0; i<nSonarReadings; i++) {
    //         Serial.print(i % 10);
    //         Serial.print(' ');
            dist += pulseIn(SONARPIN, HIGH);
        }
        dist /= nSonarReadings;
    //     Serial.println(dist);
        
        if(dist < 395)
        {
            dist = 0.0; // The sensor is piecewise,
                        // and returns a constant PWM value below about 11 inches.
        }
        else
        {
            dist = -.0306748 * (25.49 - dist); // From fitted data.
        }
        dist *= 2.54;  // Convert to centimeters.
        
        return dist;
    }

    void sonarTest()
    {
        while(true)
        {
            Serial.println(getSonarDist(32));
            delay(1);
        }
    }

    bool treadBlocked(bool right)
    {
        int blockagePanAngle = 50;
        const int blockageTiltAngle = -60;
        if(right)
        {
            Serial.print("right ");
            blockagePanAngle *= -1;
        }
        else
        {
            Serial.print("left ");
        }
            
        setPan(blockagePanAngle);
        setTilt(blockageTiltAngle);
        interruptibleDelay(4000);
        float dist = getSonarDist();
        Serial.print("ground distance: ");
        Serial.println(dist);
        setPan(0); setTilt(0);
        
        return (dist < 50);
    }
    bool leftTreadBlocked()
    {
        return treadBlocked(false);
    }
    bool rightTreadBlocked()
    {
        return treadBlocked(true);
    }
};

#endif