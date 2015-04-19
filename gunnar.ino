#include <Servo.h>
#include <MemoryFree.h>
#include <Wire.h>
#include "constants.h"
#include "gunnar.h"

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
    gunnar = Gunnar();
    gunnar.init();
    
    attachInterrupt(0, doEncoder0, RISING);
    attachInterrupt(1, doEncoder1, RISING);
}

void loop()
{
    gunnar.loop();
}
