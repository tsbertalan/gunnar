#ifndef PINDEFS_H
#define PINDEFS_H

const int BAUDRATE = 9600;

// Pin definitions:
const int PIN_ACTIVITYSWITCH = 6;
const int SONARPIN = 8;

const int TILTSERVOPIN = 9;
const int PANSERVOPIN = 10;

const int encoder0PinA = 2; // interrupt pin 0
const int encoder0PinB = 4;
const int encoder1PinA = 3; // interrupt pin 1
const int encoder1PinB = 5;

// Servo one uses pin 10.
// Servo two uses pin 9.

// The Adafruit motor shield uses these pins
// A4 and A5; still usable for i2c.


// Other constants:
const int minimumSensableDistance = 30; // [cm]

const int MAXPWMSPEED = 255;
const long SPEEDSCALE = 1024000L; // We need to keep the numerator and denominator similarly scaled.
    
const double KP = 2.0;
const double KI = 1.5;
const double KD = 0.1;
const long GOSECONDS = 32L; // Time to allow for a PID set-point search.

const int ntasks = 2;

// Sparkfun motor shield pins:
int motorPinStby = 29; //standby

//Motor A
const int motorPinPwmA = 11; // Speed control
const int motorPinAin1 = 26; //Direction
const int motorPinAin2 = 25; //Direction

//Motor B
const int motorPinPwmB = 12; //Speed control
const int motorPinBin1 = 30; //Direction
const int motorPinBin2 = 33; //Direction



#endif