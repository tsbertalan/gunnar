#include <Servo.h>
#include <gunnar.h>

Gunnar gunnar;

void setup()
{
    Serial.begin(9600);
    gunnar.init();
}

void loop()
{
    gunnar.motor1.setSpeed(100);
    gunnar.motor2.setSpeed(100);
    delay(1000);
    gunnar.motor1.setSpeed(-100);
    gunnar.motor2.setSpeed(-100);
    delay(1000);
}