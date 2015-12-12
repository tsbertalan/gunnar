#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <gunnar.h>
#include "constants.h"
#include <CmdMessenger.h>

Gunnar gunnar;
// Interrupt Service Routines

void doEncoder0()
{
    gunnar.encoder0.update();
}

void doEncoder1()
{
    gunnar.encoder1.update();
}

void setup()
{  
    Serial.begin(BAUDRATE);
    Serial.println("setup()");
    gunnar = Gunnar();  // The constructor is implicitly called anway; this line is pointless.
    gunnar.init();
    
    // Turn on pullup resistors on interrupt lines:
    pinMode(encoder0Int, INPUT_PULLUP);
    pinMode(encoder0Int, INPUT_PULLUP);
    attachInterrupt(0, doEncoder0, CHANGE);
    attachInterrupt(1, doEncoder1, CHANGE);
}

void loop()
{
//     gunnar.loop();
}
