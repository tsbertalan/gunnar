#include <Servo.h>
#include <Wire.h>
#include <MemoryFree.h>
#include <gunnar.h>

Gunnar gunnar;

void setup()
{  
    Serial.begin(BAUDRATE);
    Serial.println("setup()");
    gunnar = Gunnar();
    gunnar.init();
    gunnar.sensors.setTilt(-20);
}

void doAngle(int i)
{
    gunnar.sensors.setPan(i);
    interruptibleDelay(4);
    Serial.print(micros());
    Serial.print(" ");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(gunnar.sensors.getSonarDist(8));
}


void loop()
{
// //     Serial.println("looping");
    const int maxAngle = 60;
    for(int i=-maxAngle; i<maxAngle; i++)
    {
        doAngle(i);
    }
    
    for(int i=maxAngle; i>-maxAngle; i--)
    {
        doAngle(i);
    }
}
