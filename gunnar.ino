#include "Adafruit_MotorShield_modified.h"
#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>
#include "constants.h"
#include "gunnar.h"

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
    Serial.begin(9600);
    Serial.println("setup()");
    gunnar = Gunnar();
    gunnar.init();
    gunnar.setup();
    
    attachInterrupt(0, doEncoder0, RISING);
    attachInterrupt(1, doEncoder1, RISING);
}

void loop()
{
    gunnar.loop();
//     Serial.println("loop");
//     delay(100);
}
