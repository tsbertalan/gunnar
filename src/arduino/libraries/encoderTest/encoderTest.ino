#include "encoders.h"

Encoder encoderl;
void encoderlIsr() {
    encoderl.update();
}

void setup() {
    Serial.begin(57600);
    encoderl.init(2, 3);
    attachInterrupt(digitalPinToInterrupt(encoderl.pin_a), encoderlIsr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderl.pin_b), encoderlIsr, CHANGE);
}

void loop() {
    delay(100);
    Serial.println(encoderl.position);
}

