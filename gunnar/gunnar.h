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
        ackCallback.init(this, &Gunnar::acknowledge);
        setSpeedsCallback.init(this, &Gunnar::setSpeeds);
        handleSensorRequestCallback.init(this, &Gunnar::handleSensorRequest);

        cmdMessenger.attach(kAcknowledge, ackCallback);
        cmdMessenger.attach(kSpeedSet, setSpeedsCallback);
        cmdMessenger.attach(kSensorsRequest, handleSensorRequestCallback);

        pinMode(PIN_ACTIVITYSWITCH, INPUT);

        sensors.init();

        acknowledge();
    }

    void loopOnce() {
        cmdMessenger.feedinSerialData();
    }

    void updatePIDs() {
        controlledMotors.updatePIDs();
    }

    void updateAHRS() {
        sensors.ahrs.update();
    }

    // Callback for sending sensor data when requested.
    void acknowledge() {
        cmdMessenger.sendCmd(kAcknowledgeResponse);
    }

    void handleSensorRequest() {
        updateAHRS();
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
    Callback ackCallback;
    // This is the list of recognized commands. These can be commands that can either be sent or received.
    // In order to receive, attach a callback function to these events
    enum {
        // Commands
        kAcknowledge         , // Command to acknowledge that cmd was received
        kError               , // Command to report errors
        kSpeedSet            , // Command to set motor speeds
        kSensorsRequest      , // Command to request sensor data
        kSensorsResponse     , // Command to report sensor data
        kAcknowledgeResponse , // Command to respond to an ack request.
    };

private:
    bool headlightOn;
    int _nTurns;
    int _lastTurnTime;

    Motor* bothMtrs[2];

};


#endif
