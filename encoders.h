#ifndef ENCODER_H
#define ENCODER_H
#include "Adafruit_MotorShield_modified.h"
#include "constants.h"


class Encoder
{
public:

    // initializer : sets pins as inputs and turns on pullup resistors
    void init( int8_t PinA, int8_t PinB, Adafruit_DCMotor* assocMotor)
    {
        Serial.println("Initializing encoder.");
        pin_a = PinA;
        pin_b = PinB;
        motor = assocMotor;
        Serial.println("    Motor reference set.");
       
        // set pin a and b to be input 
        pinMode(pin_a, INPUT); 
        pinMode(pin_b, INPUT); 
       
        // and turn on pullup resistors
        digitalWrite(pin_a, HIGH);    
        digitalWrite(pin_b, HIGH);
        Serial.println("    Encoder initialized.");
    };

    
    // Call this from your interrupt function.
    void update()
    {
        // TODO: This is probably too long for an ISR.
        noInterrupts();
        
        long now = micros();
        updateDelay = now - lastUpdateTime;
        lastUpdateTime = now;
        trueUpdateDelay = updateDelay;
        
        ticks++;
                
        if(
            (digitalRead(pin_a) && digitalRead(pin_b))
            ||
            (!digitalRead(pin_a) && !digitalRead(pin_b))
            )
            position--;
        else
            position++;
        
        interrupts();

    };
    
    float getSpeed()
    {
        uint8_t motorStatus = motor->getStatus();
        if(motorStatus==BRAKE || motorStatus==RELEASE)
        {
            lastSpeedQueryResult = 0;
            return 0;
        }
        long now = micros();
        if(now - lastSpeedQueryMicros < 100L)
        {
            return lastSpeedQueryResult;
        }
        else
        {
            long here = ticks;
            long there = lastSpeedQueryTicks;
//             long here = position;
//             long there = lastSpeedQueryPosition;
            
            float speed = SPEEDSCALE * (float) (here - there) / (float) (now - lastSpeedQueryMicros);
            
            if(motorStatus == BACKWARD)
                speed *= -1;
            lastSpeedQueryTicks = here;
            
//             lastSpeedQueryPosition = here;
            lastSpeedQueryMicros = now;
            lastSpeedQueryResult = speed;
            
            return speed;
        }

    }
    
    volatile long trueUpdateDelay;
    long volatile position;
    
private:
    Adafruit_DCMotor* motor;
    volatile long updateDelay;
    volatile long lastUpdateTime;
    float lastSpeedQueryResult;
    long lastSpeedQueryMicros;
//     long lastSpeedQueryPosition;
    long lastSpeedQueryTicks;
    long volatile ticks;
    int8_t pin_a;
    int8_t pin_b;
};

#endif
