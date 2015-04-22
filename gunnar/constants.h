#ifndef PINDEFS_H
#define PINDEFS_H

const int BAUDRATE = 9600;

const uint8_t PIN_ACTIVITYSWITCH = 6;
const uint8_t SONARPIN = 8;

const uint8_t TILTSERVOPIN = 9;
const uint8_t PANSERVOPIN = 10;

// Encoders:
const uint8_t encoder0Int = 2; // interrupt pin 0
const uint8_t encoder1Int = 3; // interrupt pin 1

const uint8_t encoder0PinA = 34;
const uint8_t encoder0PinB = 36;

const uint8_t encoder1PinA = 35;
const uint8_t encoder1PinB = 37;

// Other constants:
const uint8_t minimumSensableDistance = 30; // [cm]
const int turnThresh = 120;

const uint8_t MAXPWMSPEED = 180;  // Depends on battery voltage, really.
const long SPEEDSCALE = 1024000L; // We need to keep the numerator and denominator similarly scaled.
    
const double KP = 2.0;
const double KI = 1.5;
const double KD = 0.1;

const uint8_t ntasks = 2;

// Dagu5 motor driver pins:
//Motor A
const uint8_t motorPinPwmA = 11; // Speed control
const uint8_t motorPinDirA = 26; // Direction
const uint8_t motorPinCurA = 25; // Direction

//Motor B
const uint8_t motorPinPwmB = 12; // Speed control
const uint8_t motorPinDirB = 30; // Direction
const uint8_t motorPinCurB = 33; // Direction

const uint8_t TURNSIGNALLEFTPIN = 52;
const uint8_t TURNSIGNALRIGHTPIN = 53;
const uint8_t TURNSIGNALBACKPIN = 50;
const uint8_t TURNSIGNALFORWARDPIN = 51;

const int maxTurnTime = 10000; // ms
const uint8_t sonarPeriod = 10; // ms
const uint8_t PIDperiod = 4; // ms

#endif
