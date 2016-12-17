#ifndef ENCODER_H
#define ENCODER_H
//#include "constants.h"


class Encoder {
public:

    // initializer : sets pins as inputs and turns on pullup resistors
    void init(uint8_t PinA, uint8_t PinB) {
        pin_a = PinA;
        pin_b = PinB;
        pinMode(pin_a, INPUT_PULLUP);
        pinMode(pin_b, INPUT_PULLUP);
    };

    void incrementPosition(bool clockwise) {
        // Abstract incrementPosition(true) and incrementPosition(false) so
        // they can be different for the two wheels. This only slightly
        // destroys the separation of concerns gained by having a separate
        // encoder object for each wheel.
        if(clockwise)
            position++;
        else
            position--;
    }

    // Call this from/as the interrupt service routine.
    void update() {
        // TODO: This is probably too long for an ISR.
        noInterrupts();

        ticks++;

        boolean a = digitalRead(pin_a);
        boolean b = digitalRead(pin_b);


  	    // The waves are presumed to look like this:
        //         ___     ___     ___
        // pin_a      |___|   |___|   |___
        //         1 1 0 0 1 1 0 0 1 1 0 0
        //           ___     ___     ___
        // pin_b   _|   |___|   |___|   |_
        //         0 1 1 0 0 1 1 0 0 1 1 0

        // If we get an interrupt on only half the state changes, the
        // position-change logic is simple. Forward looks like
        // 11 00 11 00 or 01 10 01 10
        // depending on which pin is tied also to interrupt, and reverse looks like
        // 01 10 01 10 or 11 00 11 00
        // W.L.G, we assume the interrupt pin is tied to pin_a.
//
//      if(a == b) {
//          incrementPosition(true);
//      } else {
//          incrementPosition(false);
//      }

        // If we get an interrupt on every state change,
        // we need more elaborate position increment/decrement logic.
        // forward looks like  10 11 01 00 10 ... = 2 3 1 0 2 ...
        // backward looks like 00 01 11 10 00 ... = 0 1 3 2 0 ...
        // So, we need to keep track of the previous state.
        // If this proves to be too much computational load (both because
        // of the increased temporal resolution, and the increased tracking
        // complexity), I might bypass the encoder-mixing circuit on the
        // motor board, tie the interrupts to pin_a for both encoders,
        // and just use the simpler code above.

        uint8_t newStatus = b + 2*a;
        switch(previousStatus) {
        case 0 : // 00
            if(newStatus == 2) // 10
                incrementPosition(true);
            else {
                if(newStatus == 1) // 01
                    incrementPosition(false);
            }
            break;
        case 1 : // 01
            if(newStatus == 0)  // 00
                incrementPosition(true);
            else {
                if(newStatus == 3)  // 11
                    incrementPosition(false);
            }
            break;
        case 2 : // 10
            if(newStatus == 3)  // 11
                incrementPosition(true);
            else {
                if(newStatus == 0)  // 00
                    incrementPosition(false);
            }
            break;
        case 3 : // 11
            if(newStatus == 1) // 01
                incrementPosition(true);
            else {
                if(newStatus == 2) // 10
                    incrementPosition(false);
            }
            break;
        default :
            break; // Should never reach here.
        }

        previousStatus = newStatus;


        interrupts();
    };

    volatile long trueUpdateDelay;
    signed long volatile position;
    signed long volatile ticks;

    int8_t pin_a;
    int8_t pin_b;

private:
    uint8_t previousStatus;
    uint8_t previousPreviousStatus;
    float lastSpeedQueryResult;
    long lastSpeedQueryMicros;
    long lastSpeedQueryTicks;
};

#endif
