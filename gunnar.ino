#define TOM_DEBUG // Uncomment to enable debugging serial prints.
#include "Adafruit_MotorShield_modified.h"
#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>
#include "motorPIDcontrol.h"
#include "stats.h"
#include "pinDefinitions.h"
#include "encoders.h"
#include "taskDriver.h"



// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor* motor1 = AFMS.getMotor(1);
Adafruit_DCMotor* motor2 = AFMS.getMotor(2);

Adafruit_DCMotor* bothMtrs[] = {motor1, motor2};

Encoder encoder0(encoder0PinA, encoder0PinB, motor1);
Encoder encoder1(encoder1PinA, encoder1PinB, motor2);

#include "motorPIDcontrol.h"

bool running = false;



unsigned long lastTurn = 0;
 
const int PANOFFSET = 10; // Larger is further to the left.
const int TILTOFFSET = 8; // Larger is further down.
const int NEUTRALPAN = -PANOFFSET;
const int NEUTRALTILT = -TILTOFFSET;




bool leftTreadBlocked()
{
    return treadBlocked(false);
}
bool rightTreadBlocked()
{
    return treadBlocked(true);
}
bool treadBlocked(bool right)
{
    int blockagePanAngle = 50;
    const int blockageTiltAngle = -60;
    if(right)
    {
        Serial.print("right ");
        blockagePanAngle *= -1;
    }
    else
    {
        Serial.print("left ");
    }
        
    setPan(blockagePanAngle);
    setTilt(blockageTiltAngle);
    interruptibleDelay(4000);
    float dist = getSonarDist();
    Serial.print("ground distance: ");
    Serial.println(dist);
    setPan(0); setTilt(0);
    
    return (dist < 50);
}
    
const int stepDelay = 10;
int nTurns = 0;





            


// MAIN PROGRAM

void setup() {
    visionSetup();
    encodersSetup();
    motorPIDcontrolSetup();
    
    pinMode(PIN_ACTIVITYSWITCH, INPUT);
        
    Serial.begin(9600);           // set up Serial library at 9600 bps

    AFMS.begin(1600);

    // Set the speed to start, from 0 (off) to 255 (max speed)

    motor1->run(RELEASE);
    motor2->run(RELEASE);

    //   delay(2000);
    //   go(32);
    //   stop();
    taskerSetup();
}

void loop()
{
    taskerLoop();
}

// void loop() {
//     
//     if(digitalRead(PIN_ACTIVITYSWITCH) == HIGH)
//     {
//         
//         unsigned long now = micros()*1000L;
//         if( running && (now - lastTurn > 16000) )
//         {
//             // If it's been a while since the last turn, just turn now anyway.
//             running = false; // We'll do the decision while moving. Possibly bad.
//             decideTurn();
//         }
//         else
//         {
//             float dist = getSonarDist(8);
//             Serial.print("sighted distance: "); Serial.println(dist);
// //             dist = 4; // prevents forward running.
// 
//            
//         }
//     }
//     else
//     {
//         stop();
//         disableServos();
//     }
//     interruptibleDelay(stepDelay);
// 
// //     sonarTest();
//     
//     #ifdef TOM_DEBUG
//         Serial.print("v0 = ");
//         Serial.println(encoder0.getSpeed());
// //         Serial.print("v1 = ");
// //         Serial.println(encoder1.getSpeed());
//     #endif
// }
