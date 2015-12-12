#ifndef GUNNAR_H
#define GUNNAR_H
#include "Arduino.h"
#include "motorPIDcontrol.h"
#include "constants.h"
#include "encoders.h"
#include "vision.h"

// For CmdMessenger:
#include <CmdMessenger.h>
#include <inttypes.h>
#include <Arduino.h>

// Consolidate as many globals as possible in a singleton robot.
class Gunnar {
public:
    Gunnar() {
    }

    void init() {
        for(uint8_t i=0; i<3; i++) {
            controlledMotors.signal.forward();
            interruptibleDelay(50);
            controlledMotors.signal.backward();
            interruptibleDelay(50);
            controlledMotors.signal.stop();
            interruptibleDelay(50);
        }
        controlledMotors.signal.none();
        motor1 = Motor();
        motor1.init(MOTORLEFT);
        motor2 = Motor();
        motor2.init(MOTORRIGHT);
        bothMtrs[0] = &motor1;
        bothMtrs[1] = &motor2;

        encoder0.init(encoder0PinA, encoder0PinB, NULL);
        encoder1.init(encoder1PinA, encoder1PinB, NULL);

        controlledMotors.init(&motor1, &motor2, &encoder0, &encoder1, &sensors);
        motor1.stop();
        motor2.stop();

        // Set up message handling.
        // Attach to the default Serial port.
        cmdMessenger.init(Serial);

        // Adds newline to every command
        cmdMessenger.printLfCr();

        // Attach actual callbacks to Callback objects, and attach them to the cmdMessenger.
        setSpeedsCallback.init(this, &Gunnar::setSpeeds);
        handleSensorRequestCallback.init(this, &Gunnar::handleSensorRequest);

        cmdMessenger.attach(kSpeedSet, setSpeedsCallback);
        cmdMessenger.attach(kSensorsRequest, handleSensorRequestCallback);

        pinMode(PIN_ACTIVITYSWITCH, INPUT);

        sensors.init();
    }

    void loopOnce() {
        cmdMessenger.feedinSerialData();
    }

    void checkSonar() {
        bool activitySwitch = checkActivitySwitch();
        activitySwitch = true;
        if(activitySwitch) {
//             Serial.println("Activity switch .");
            const int backoff = 30;
            float dist = sensors.getSonarDist(8);
            if (!keyboardControl) {
//                 Serial.println("Running autonomous wander routine.");
//                 Serial.print(F("sighted distance: ")); Serial.println(dist);
                if(dist < turnThresh) {
                    if(dist < minimumSensableDistance) {
                        controlledMotors.stop();
                        controlledMotors.go(-backoff);
                    } else { // We're not *super* close.
                        if(_nTurns > 4) {
                            _nTurns = 0;
                            controlledMotors.stop();
                            controlledMotors.go(-backoff);
                        } else { // We haven't turned very many times.
                            Serial.println("Obstacle sighted. Turning.");
                            decideTurn();
                            _nTurns++;
                        }
                    }
                } else { // dist >= 60
                    _nTurns = 0;
                    controlledMotors.go(dist/4);
                }
            }
        } else {
            Serial.println("Activity switch is off.");
            sensors.disableServos();
            controlledMotors.stop();
        }
    }

    void updatePIDs() {
        controlledMotors.updatePIDs();
    }

    void updateAHRS() {
        sensors.ahrs.update();
    }

    void decideTurn(int absAngle=-1) {
        // If We're currently in the process of turning,
        // only initiate a new turn if the current turn error has gone below
        // threshold.
        int timeTurning = millisViaMicros() - _lastTurnTime;
        boolean doTurn = true;
        if(controlledMotors.isTurning()) {
            doTurn = false;

            Serial.print(F("We've been turning for the last "));
            Serial.print(timeTurning);
            Serial.println(F(" ms."));
            if(timeTurning > maxTurnTime) {
                Serial.println(F("Turning timeout."));
                doTurn = true;
            }

            if(controlledMotors.getEucError() < EUCCONTROLERRORTHRESH) {
                Serial.println(F("Turning goal reached. New turn innitiated."));
            }
        }

        if(doTurn) {
            controlledMotors.stop();
            _lastTurnTime = millisViaMicros();

            const int delayBeforeMeasurement = 3;
            const int nMeasurements = 4;
            sensors.setTilt(10); // Look up slightly.

            sensors.setPan(-45);
            interruptibleDelay(delayBeforeMeasurement);
            float rdist = sensors.getSonarDist(nMeasurements);
            //     Serial.print("rdist: ");
            //     Serial.println(rdist);

            sensors.setPan(45);
            interruptibleDelay(delayBeforeMeasurement);
            float ldist = sensors.getSonarDist(nMeasurements);
            //     Serial.print("ldist: ");
            //     Serial.println(ldist);
            sensors.setPan(0);
            sensors.setTilt(0);
            interruptibleDelay(delayBeforeMeasurement*10);

//             // Arduino's sprintf doesn't have %f.
//             // stackoverflow.com/questions/27651012
//             char msg[64];
//             char rtmp[8];
//             char ltmp[8];
//             dtostrf(rdist, 6, 2, rtmp);
//             dtostrf(ldist, 6, 2, ltmp);

            if( absAngle == -1) {
                float C = sqrt(ldist*ldist + rdist*rdist);
                absAngle = 180.0 / 3.14159 * asin(min(ldist, rdist) / C);
            }
            absAngle = abs(absAngle);

            int angle;
            if(max(rdist, ldist) < minimumSensableDistance) {
                Serial.println("Too close to wall to decide which way to turn.");
                angle = 180;
            } else {
                if( rdist < ldist ) {

//                     sprintf(msg, "rdist=%s < ldist=%s -- ", rtmp, ltmp);
//                     Serial.print(msg);
                    angle = absAngle;
                } else {
//                     sprintf(msg, "rdist=%s >= ldist=%s -- ", rtmp, ltmp);
//                     Serial.print(msg);
                    angle = -absAngle;
                }
            }

            Serial.print("angle=");
            Serial.println(angle);

            sensors.disableServos();

            controlledMotors.turn(angle);
        }
    }

    // Callback for sending sensor data when requested.
    void handleSensorRequest() {
        cmdMessenger.sendCmdStart(kSensorsResponse);
        cmdMessenger.sendCmdArg(millis());

        cmdMessenger.sendCmdArg(sensors.getSonarDist());

        cmdMessenger.sendCmdArg(sensors.ahrs.getHeading());
        cmdMessenger.sendCmdArg(sensors.ahrs.orientation.heading);
        cmdMessenger.sendCmdArg(sensors.ahrs.orientation.roll);
        cmdMessenger.sendCmdArg(sensors.ahrs.orientation.pitch);

        cmdMessenger.sendCmdArg(sensors.ahrs.orientation.x);
        cmdMessenger.sendCmdArg(sensors.ahrs.orientation.y);
        cmdMessenger.sendCmdArg(sensors.ahrs.orientation.z);

        cmdMessenger.sendCmdArg(sensors.ahrs.orientation.v[0]);
        cmdMessenger.sendCmdArg(sensors.ahrs.orientation.v[1]);
        cmdMessenger.sendCmdArg(sensors.ahrs.orientation.v[2]);

        cmdMessenger.sendCmdArg(encoder0.position);
        cmdMessenger.sendCmdArg(motor1.getSpeed());
        cmdMessenger.sendCmdArg(motor1.getStatus());

        cmdMessenger.sendCmdArg(encoder1.position);
        cmdMessenger.sendCmdArg(motor2.getSpeed());
        cmdMessenger.sendCmdArg(motor2.getStatus());

        cmdMessenger.sendCmdArg(controlledMotors.isTurning());

        cmdMessenger.sendCmdEnd();
    }

    void saySpeeds() {
        Serial.print("Motor speeds are (");
        Serial.print(motor1.getSpeedSigned());
        Serial.print(", ");
        Serial.print(motor2.getSpeedSigned());
        Serial.println(").");
    }

    void checkSerialForCommands() {
        char charRead;
        const int TURNSPEEDINC = 8;
        const int STRAIGHTSPEEDINC = TURNSPEEDINC;
        if (Serial.available()) {
            charRead = Serial.read();
            /*ECHO the value that was read, back to the serial port. */
            //        Serial.println("");
            //        Serial.println("Arduino got this input:");
            //        Serial.println(charRead);
            if (charRead == 'w') {
                Serial.print("w: forward");
                int speed1 = constrain(motor1.getSpeedSigned() + STRAIGHTSPEEDINC, -255, 255);
                int speed2 = constrain(motor2.getSpeedSigned() + STRAIGHTSPEEDINC, -255, 255);
                motor1.setSpeed(speed1);
                motor2.setSpeed(speed2);
                Serial.print(" (set to ");
                Serial.print(speed1);
                Serial.print(", ");
                Serial.print(speed2);
                Serial.println(".)");
//                 controlledMotors.go(12);
            }

            else if (charRead == 's') {
                Serial.println("s: reverse");
                motor1.setSpeed(constrain(motor1.getSpeedSigned() - STRAIGHTSPEEDINC, -255, 255));
                motor2.setSpeed(constrain(motor2.getSpeedSigned() - STRAIGHTSPEEDINC, -255, 255));
//                 controlledMotors.go(-12);
            }

            else if (charRead == 'a') {
                Serial.print("a: left");
                int speed1 = constrain(motor1.getSpeedSigned() - TURNSPEEDINC, -255, 255);
                int speed2 = constrain(motor2.getSpeedSigned() + TURNSPEEDINC, -255, 255);
                motor1.setSpeed(speed1);
                motor2.setSpeed(speed2);
                Serial.print(" (set to ");
                Serial.print(speed1);
                Serial.print(", ");
                Serial.print(speed2);
                Serial.println(".)");
//                 controlledMotors.turn(-90);
            }

            else if (charRead == 'd') {
                Serial.println("d: right");
                motor1.setSpeed(constrain(motor1.getSpeedSigned() + TURNSPEEDINC, -255, 255));
                motor2.setSpeed(constrain(motor2.getSpeedSigned() - TURNSPEEDINC, -255, 255));
//                 controlledMotors.turn(90);
            }

            else if (charRead == ' ') {
                motor1.stop();
                motor2.stop();
                Serial.println("space: stop");
                controlledMotors.go(0);
                controlledMotors.stop();
            } else if (charRead == 'f') {
                // Turn on headlight.
                if (headlightOn) {
                    controlledMotors.signal.none();
                    Serial.println("f: headlight off");
                } else {
                    controlledMotors.signal.forward();
                    Serial.println("f: headlight on");
                }
                headlightOn = !headlightOn;
            }
            saySpeeds();
        }
    }

    // Callback function sets the motor speeds.
    void setSpeeds() {
        // Retreive first parameter as float
        float leftSpeed = cmdMessenger.readFloatArg();

        // Retreive second parameter as float
        float rightSpeed = cmdMessenger.readFloatArg();

        motor1.setSpeed(leftSpeed);
        motor2.setSpeed(rightSpeed);
    }

    // Called when a received command has no attached function
    void unknownCommandCallback() {
        cmdMessenger.sendCmd(kError, "Command without attached callback");
    }

    Encoder encoder0;
    Encoder encoder1;
    Sensors sensors;
    ControlledMotors controlledMotors;
    Motor motor1;
    Motor motor2;

    CmdMessenger cmdMessenger;
    Callback setSpeedsCallback;
    Callback handleSensorRequestCallback;
    // This is the list of recognized commands. These can be commands that can either be sent or received.
    // In order to receive, attach a callback function to these events
    enum {
        // Commands
        kAcknowledge         , // Command to acknowledge that cmd was received
        kError               , // Command to report errors
        kSpeedSet            , // Command to set motor speeds
        kSensorsRequest      , // Command to request sensor data
        kSensorsResponse     , // Command to report sensor data
    };

private:
    bool headlightOn;
    int _nTurns;
    int _lastTurnTime;

    Motor* bothMtrs[2];

};


#endif
