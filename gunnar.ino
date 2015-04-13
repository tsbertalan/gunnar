#include "Adafruit_MotorShield_modified.h"
#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>
#include "motorPIDcontrol.h"
#include "stats.h"
#include "pinDefinitions.h"
#include "encoders.h"
#include "taskDriver.h"
#include "motorPIDcontrol.h"

bool running = false;



unsigned long lastTurn = 0;
 






    
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
