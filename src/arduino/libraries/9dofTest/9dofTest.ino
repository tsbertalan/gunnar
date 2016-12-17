#include "ahrs.h"

AHRS ahrs;

void setup() {
    Serial.begin(57600);
    ahrs.init();
}

void loop() {
    ahrs.update();
    Serial.print(F("Heading: "));
    Serial.println(ahrs.getHeading());
    delay(100);
}

