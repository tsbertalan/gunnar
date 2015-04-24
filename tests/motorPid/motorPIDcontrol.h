#ifndef MOTORPIDCONTROL_H
#define MOTORPIDCONTROL_H
#include <PID_v1.h>

const double KP = 2.0;
const double KI = 1.5;
const double KD = .1;

double leftMotorControlledSpeed;
double leftMotorSetPoint;

double rightMotorControlledSpeed;
double rightMotorSetPoint;

extern PID leftPID, rightPID;

void controlMotorSpeeds(double left, double right);
#endif
