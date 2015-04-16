#ifndef PINDEFS_H
#define PINDEFS_H

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
const long SPEEDSCALE = 256000L; // We need to keep the numerator and denominator similarly scaled.
    
const double KP = 2.0;
const double KI = 1.5;
const double KD = 0.1;

const int ntasks = 2;


#endif