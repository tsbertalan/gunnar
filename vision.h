#ifndef VISION_H
#define VISION_H
#include <Servo.h>
#include "pinDefinitions.h"

const int PANOFFSET = 10; // Larger is further to the left.
const int TILTOFFSET = 8; // Larger is further down.
const int NEUTRALPAN = -PANOFFSET;
const int NEUTRALTILT = -TILTOFFSET;

Servo *servoTilt;
Servo *servoPan;

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