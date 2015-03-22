#include <Servo.h>

const int SONARPIN = 2;

const int TILTSERVOPIN = 9;
const int PANSERVOPIN = 10;
Servo servoTilt;
Servo servoPan;

const int PANOFFSET = 5; // Larger is further to the left.
const int TILTOFFSET = 8; // Larger is further down. 

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
    servoPan.write(pos+90+PANOFFSET);
//     Serial.print("setting pan servo to ");
//     Serial.println(pos+90);
}


void setTilt(int pos)
{
    // -90 <= pos < 90
    servoTilt.write(-pos+90+TILTOFFSET);
//     Serial.print("setting tilt servo to ");
//     Serial.println(-pos+90);
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
