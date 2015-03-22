#include <Servo.h>

const int TILTSERVOPIN = 9;
const int PANSERVOPIN = 10;
Servo servoTilt;
Servo servoPan;

const int PANOFFSET = 5; // Larger is further to the left.
const int TILTOFFSET = 8; // Larger is further down. 

void visionSetup()
{
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