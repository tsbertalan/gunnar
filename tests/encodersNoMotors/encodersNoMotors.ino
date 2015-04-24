#include <Arduino.h>  // I don't know why this is necessary.
#include <Wire.h>
#include "constants.h"
#include "utils.h"
#include "encoders.h"
#include "motors.h"


// GLOBALS
Encoder encoder0;
Encoder encoder1;

void doEncoder0()
{
    encoder0.update();
}

void doEncoder1()
{
    encoder1.update();
}

const boolean explore = false;
// Test the motors and encoders.
void setup() {
    if(explore)
    {
        Serial.begin(9600);
            
        boolean a = LOW;
        boolean b = LOW;
        
        uint8_t newStatus = a + 2*b;
        
        // backward looks like 0, 1, 2, 3
        // forward  looks like 0, 3, 2, 1
        
        int position = 0;
        uint8_t waveStatus = 2;
        
        switch(waveStatus)
        {
            case 0 :
                Serial.println("case 0");
                if(newStatus == 1)
                    position--;
                else
                    position++;
                break;
            case 1 :
                Serial.println("case 1");
                if(newStatus == 2)
                    position--;
                else
                    position++;
                break;
            case 2 :
                Serial.println("case 2");
                if(newStatus == 3)
                    position--;
                else
                    position++;
                break;
            case 3 :
                Serial.println("case 3");
                if(newStatus == 1)
                    position--;
                else
                    position++;
                break;
            default :
                break; // Should never reach here.
        }
        
        Serial.print("newStatus=");
        Serial.println(newStatus);
        Serial.print("position=");
        Serial.println(position);
    }
    else
    {
        Serial.begin(9600);
        
        encoder0.init(encoder0PinA, encoder0PinB, NULL);
        encoder1.init(encoder1PinA, encoder1PinB, NULL);
            
        // Turn on pullup resistors on interrupt lines:
        pinMode(2, INPUT_PULLUP);
        pinMode(3, INPUT_PULLUP);
        attachInterrupt(0, doEncoder0, CHANGE);
        attachInterrupt(1, doEncoder1, CHANGE);
        
        pinMode(PIN_ACTIVITYSWITCH, INPUT);
    }
}


void loop()
{
    if(explore)
    {
    }
    else
    {
        Serial.print(micros());
        Serial.print(", ");
        Serial.print(encoder0.getSpeed());
        Serial.print(", ");
        Serial.print(encoder1.getSpeed());
        Serial.print(", ");
        Serial.print(encoder0.position);
        Serial.print(", ");
        Serial.print(encoder1.position);
        Serial.print(", ");
        Serial.print(encoder0.trueUpdateDelay);
        Serial.print(", ");
        Serial.print(encoder1.trueUpdateDelay);
        Serial.println("");
        delayMicroseconds(1000000);
    }
}
