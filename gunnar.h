#ifndef GUNNAR_H
#define GUNNAR_H
#include "Adafruit_MotorShield_modified.h"

// Consolidate all the globals in a singleton robot.

class Gunnar
{
public:
    void init()
    {
        AFMS = Adafruit_MotorShield();
        motor1 = AFMS.getMotor(1);
        motor2 = AFMS.getMotor(2);
        bothMtrs[0] = motor1;
        bothMtrs[1] = motor2;
        
        encoder0 = Encoder(encoder0PinA, encoder0PinB, motor1);
        encoder1 = Encoder(encoder1PinA, encoder1PinB, motor2);
    }
    void live(){
        
    
private:
    Adafruit_Motorshield AFMS;
    Adafruit_DCMotor* motor1;
    Adafruit_DCMotor* motor2;
    Adafruit_DCMotor* bothMtrs[2];
    
    Encoder encoder0;
    Encoder encoder1;
}
#endif