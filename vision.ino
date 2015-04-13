#include "vision.h"

void visionSetup()
{
    pinMode(SONARPIN, INPUT);
    
    servoTilt.attach(TILTSERVOPIN);
    servoPan.attach(PANSERVOPIN);
    
    setPan(0);
    setTilt(0);
}

void setPan(int pos)
{
    // -90 <= pos < 90
    if(!servoPan.attached())
    {
        servoPan.attach(PANSERVOPIN);
    }
    servoPan.write(pos+90+PANOFFSET);
}

void setTilt(int pos)
{
    // -90 <= pos < 90
    if(!servoTilt.attached())
    {
        servoTilt.attach(TILTSERVOPIN);
    }
    servoTilt.write(-pos+90+TILTOFFSET);
}

void disablePan()
{
    setPan(0);
    if(servoPan.attached())
        servoPan.detach();
}

void disableTilt()
{
    setTilt(0);
    if(servoTilt.attached())
        servoTilt.detach();
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
