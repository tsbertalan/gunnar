#ifndef PINDEFS_H
#define PINDEFS_H

const int BAUDRATE = 19200;

const uint8_t PIN_ACTIVITYSWITCH = 6;
const uint8_t SONARPIN = 8;

const uint8_t TILTSERVOPIN = 9;
const uint8_t PANSERVOPIN = 10;

const int PANOFFSET = 10; // Larger is further to the left.
const int TILTOFFSET = 10; // Larger is further down.

// Encoders:
const uint8_t encoder0Int = 2; // interrupt pin 0 = digial pin 2
const uint8_t encoder1Int = 3; // interrupt pin 1 = digial pin 3

const uint8_t encoder0PinA = 4;
const uint8_t encoder0PinB = 5;

const uint8_t encoder1PinA = 6;
const uint8_t encoder1PinB = 7;

// Other constants:
const uint8_t minimumSensableDistance = 30; // [cm]
const int turnThresh = 120;

const uint8_t MAXPWMSPEED = 180;  // Depends on battery voltage, really.
const long SPEEDSCALE = 1024000L; // We need to keep the numerator and denominator similarly scaled.

const double KP = 2.0;
const double KI = 1.5;
const double KD = 0.1;


// Dagu5 motor driver pins:
//Motor A
const uint8_t motorPinPwmA = 11; // Speed control
const uint8_t motorPinDirA = 22; // Direction
const uint8_t motorPinCurA = A0; // Current monitoring

//Motor B
const uint8_t motorPinPwmB = 12; // Speed control
const uint8_t motorPinDirB = 45; // Direction
const uint8_t motorPinCurB = A1; // Current monitoring

const uint8_t TURNSIGNALLEFTPIN = 52;
const uint8_t TURNSIGNALRIGHTPIN = 53;
const uint8_t TURNSIGNALBACKPIN = 50;
const uint8_t TURNSIGNALFORWARDPIN = 51;

const int maxTurnTime = 10000; // ms

// Task driver
const uint8_t ntasks = 5;
const int sonarPeriod = 32; // ms
#define NUMHEADINGHISTS 25
const int PIDperiod = 100; // ms
const int ahrsPeriod = 4; // ms, =PIDperiod/NUMHEADINGHISTS
const int sendDataPeriod = 100;  // ms
const int checkSerialForCommandsPeriod = 10;  // ms

const double EUCCONTROLERRORTHRESH = 64.0; // Euclidean distance threshold for stopping PID.
const uint8_t CONTROLLOOPMICROS = 4; // Delay for position PID loop.
const long MAXCONTROLLOOPMICROS = 10L*1000L*1000L;  // Longest time to do position PID, in microseconds.

const bool keyboardControl = true;

#endif
