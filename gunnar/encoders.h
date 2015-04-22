#ifndef ENCODER_H
#define ENCODER_H
#include "constants.h"
#include "motors.h"


class Encoder
{
public:

    // initializer : sets pins as inputs and turns on pullup resistors
    void init(uint8_t PinA, uint8_t PinB, Motor* assocMotor)
    {
        Serial.println("Initializing encoder.");
        pin_a = PinA;
        pin_b = PinB;
        motor = assocMotor;
        Serial.println("    Motor reference set.");
       
        // set pin a and b to be input 
        pinMode(pin_a, INPUT_PULLUP); 
        pinMode(pin_b, INPUT_PULLUP); 
       
//         // and turn on pullup resistors
//         digitalWrite(pin_a, HIGH);    
//         digitalWrite(pin_b, HIGH);
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
        
        boolean a = digitalRead(pin_a);
        boolean b = digitalRead(pin_b);
        
        uint8_t newStatus = a + 2*b;
        
        // forward looks like  0, 1, 2, 3
        // backward looks like 0, 3, 2, 1
        
        switch(status)
        {
            case 0 :
                if(newStatus == 1)
                    position++;
                else
                    position--;
            case 1 :
                if(newStatus == 2)
                    position++;
                else
                    position--;
            case 2 :
                if(newStatus == 3)
                    position++;
                else
                    position--;
            case 3 :
                if(newStatus == 1)
                    position++;
                else
                    position--;
            default :
                break; // Should never reach here.
        }

        status = newStatus;
        
        interrupts();
    };
    
    float getSpeed()
    {
        uint8_t motorStatus = motor->getStatus();
        if(motorStatus==MOTORSTOP)
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
            
            float speed = abs(SPEEDSCALE * (float) (here - there) / (float) (now - lastSpeedQueryMicros));
            
            if(motorStatus == MOTORBACKWARD)
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
    long volatile ticks;
    Motor* motor;
    volatile long updateDelay;
    volatile long lastUpdateTime;
    float lastSpeedQueryResult;
    long lastSpeedQueryMicros;
//     long lastSpeedQueryPosition;
    long lastSpeedQueryTicks;
    int8_t pin_a;
    int8_t pin_b;
    uint8_t waveStatus;
};

#endif
