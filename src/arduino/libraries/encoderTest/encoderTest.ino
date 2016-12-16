#include "pins_arduino.h"

class Encoder {
public:
    volatile long ticks;
    char pina;
    char pinb;
    volatile int last;
    volatile int lastLast;
    boolean fullRes;

    void init(int a, int b) {
        pina = a;
        pinb = b;
        pinMode(a, INPUT_PULLUP);
        pinMode(b, INPUT_PULLUP);
        fullRes = false;
    }

    void fullResISR() {
        int aval = digitalRead(pina);
        int bval = digitalRead(pinb);

        // I can't do bitwise arithmetic.
        int now = aval + 2 * bval;

        switch (now) {
        case 0:
            switch (last) {
            case 1:
                ticks++;
            case 2:
                ticks--;
            }
        case 1:
            switch (last) {
            case 3:
                ticks++;
            case 0:
                ticks--;
            }
        case 3:
            switch (last) {
            case 2:
                ticks++;
            case 1:
                ticks--;
            }
        case 2:
            switch (last) {
            case 0:
                ticks++;
            case 3:
                ticks--;
            }
        }

        lastLast = last;
        last = now;
    }

    void partialResISR() {
        int aval = digitalRead(pina);
        int bval = digitalRead(pinb);

        if(aval == bval) {
            ticks--;
        } else {
            ticks++;
        }
    }

    void isr() {
        if(fullRes) {
            fullResISR();
        } else {
            partialResISR();
        }
    }

};

Encoder encoderl;
void encoderlIsr() {
    encoderl.isr();
}

#define BAUDRATE     57600
void setup() {
    Serial.begin(BAUDRATE);
    encoderl.init(2, 3);

//    // Full resolution:
//    encoderl.fullRes = true;
//    attachInterrupt(digitalPinToInterrupt(encoderl.pina), encoderlIsr, CHANGE);
//    attachInterrupt(digitalPinToInterrupt(encoderl.pinb), encoderlIsr, CHANGE);
  
    // Half resolution:
    encoderl.fullRes = false;
    attachInterrupt(digitalPinToInterrupt(encoderl.pina), encoderlIsr, CHANGE);

//    // Quarter resolution:
//    encoderl.fullRes = false;
//    attachInterrupt(digitalPinToInterrupt(encoderl.pina), encoderlIsr, RISING);
}

void loop() {
    delay(1000);
    Serial.print(encoderl.ticks);
    Serial.print(" ");
    Serial.print(encoderl.lastLast);
    Serial.print(" -> ");
    Serial.print(encoderl.last);
    Serial.print(" = ");
    Serial.print(digitalRead(encoderl.pina));
    Serial.print(" + 2*");
    Serial.print(digitalRead(encoderl.pinb));
    Serial.println();
    
}
