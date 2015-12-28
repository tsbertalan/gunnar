#ifndef ENCODER_H
#define ENCODER_H
#include "constants.h"
#include "motors.h"


class Encoder {
public:

    // initializer : sets pins as inputs and turns on pullup resistors
    void init(uint8_t PinA, uint8_t PinB, Motor* assocMotor) {
        pin_a = PinA;
        pin_b = PinB;
        motor = assocMotor;

        // set pin a and b to be input
        pinMode(pin_a, INPUT_PULLUP);
        pinMode(pin_b, INPUT_PULLUP);

//         // and turn on pullup resistors
//         digitalWrite(pin_a, HIGH);
//         digitalWrite(pin_b, HIGH);
    };

    // Call this from your interrupt function.
    void update() {
        // TODO: This is probably too long for an ISR.
        noInterrupts();

        long now = micros();
        updateDelay = now - lastUpdateTime;
        lastUpdateTime = now;
        trueUpdateDelay = updateDelay;

        ticks++;

        boolean a = digitalRead(pin_a);
        boolean b = digitalRead(pin_b);

        uint8_t newStatus = b + 2*a;

//         ___     ___     ___
// pin_a      |___|   |___|   |___
//         1 1 0 0 1 1 0 0 1 1 0 0
//           ___     ___     ___
// pin_b   _|   |___|   |___|   |_
//
//         0 1 1 0 0 1 1 0 0 1 1 0
//
//      forward looks like  10 11 01 00 10 ... = 2 3 1 0 2 ...
//      backward looks like 00 01 11 10 00 ... = 0 1 3 2 0 ...

        switch(waveStatus) {
        case 0 : // 00
            if(newStatus == 2) // 10
                position++;
            else {
                if(newStatus == 1) // 01
                    position--;
            }
            break;
        case 1 : // 01
            if(newStatus == 0)  // 00
                position++;
            else {
                if(newStatus == 3)  // 11
                    position--;
            }
            break;
        case 2 : // 10
            if(newStatus == 3)  // 11
                position++;
            else {
                if(newStatus == 0)  // 00
                    position--;
            }
            break;
        case 3 : // 11
            if(newStatus == 1) // 01
                position++;
            else {
                if(newStatus == 2) // 10
                    position--;
            }
            break;
        default :
            break; // Should never reach here.
        }

        waveStatus = newStatus;

        interrupts();
    };

    float getSpeed() {
        uint8_t motorStatus = motor->getStatus();
        if(motorStatus==MOTORSTOP) {
            lastSpeedQueryResult = 0;
            return 0;
        }
        long now = micros();
        if(now - lastSpeedQueryMicros < 100L) {
            return lastSpeedQueryResult;
        } else {
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
