#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <Wire.h>
#include <Servo.h>
#include <gunnar.h>

Gunnar gunnar;

// Interrupt Service Routines
void doEncoder0()
{
    gunnar.encoder0.update();
}

void doEncoder1()
{
    gunnar.encoder1.update();
}

void setup()
{
    Serial.begin(BAUDRATE);
    
    gunnar = Gunnar();  // The constructor is implicitly called anway; this line is pointless.
    gunnar.init();

    // Turn on pullup resistors on interrupt lines:
    pinMode(encoder0Int, INPUT_PULLUP);
    pinMode(encoder0Int, INPUT_PULLUP);
    attachInterrupt(0, doEncoder0, CHANGE);
    attachInterrupt(1, doEncoder1, CHANGE);
    Serial.println("Done with Gunnar.setup().");
}

void loop()
{
    while(true) {
        gunnar.checkSerialForCommands();
    }
//     gunnar.loop();
}
