#include "encoders.h"

Encoder encoderl;
void encoderlIsr() {
    encoderl.update();
}
Encoder encoderr;
void encoderrIsr() {
    encoderr.update();
}

void setup() {
    Serial.begin(57600);
    encoderl.init(2, 3);
    attachInterrupt(digitalPinToInterrupt(encoderl.pin_a), encoderlIsr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderl.pin_b), encoderlIsr, CHANGE);
    encoderr.init(20, 21);
    attachInterrupt(digitalPinToInterrupt(encoderr.pin_a), encoderrIsr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderr.pin_b), encoderrIsr, CHANGE);
}

void loop() {
    delay(100);
    Serial.print(encoderl.position);
    Serial.print(" ");
    Serial.print(encoderr.position);
    Serial.println("");
}

