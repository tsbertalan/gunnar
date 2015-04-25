#include <Wire.h>
#include <Servo.h>
#include <gunnar.h>

Gunnar gunnar;

// Test the motors, encoders, and PID control of position.
void setup() {
    
    Serial.begin(9600);
    gunnar.init();
    gunnar.controlledMotors.stop();
}


void loop()
{
    gunnar.controlledMotors.go(100);
    gunnar.controlledMotors.go(-200);
    gunnar.controlledMotors.go(600);
}




