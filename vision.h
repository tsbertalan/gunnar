#ifndef VISION_H
#define VISION_H
#include <Servo.h>

Servo servoTilt;
Servo servoPan;

void visionSetup();
void setPan(int);
void setTilt(int);
void disablePan();
void disableTilt();
void disableServos();
float getSonarDist();
float getSonarDist(int);
void sonarTest();

#endif