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

struct SensorResponse {
    unsigned long ms;
    float heading, roll, pitch;
    float x, y, z;
    unsigned long enc1pos, enc2pos;
    signed long enc1spd, enc2spd;
    unsigned int enc1stat, enc2stat;
    bool isTurning;
};

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
//         acknowledge();

        updateAHRS();
        // one byte
        cmdMessenger.sendCmdStart(kSensorsResponse);
        
        // Assemble the response struct (45 bytes).
        SensorResponse response;
        response.ms = millis();
        response.heading = sensors.ahrs.orientation.heading;
        response.roll= sensors.ahrs.orientation.roll;
        response.pitch = sensors.ahrs.orientation.pitch;
        
        response.x = sensors.ahrs.orientation.x;
        response.y= sensors.ahrs.orientation.y;
        response.z = sensors.ahrs.orientation.z;
        
        response.enc1pos = encoder0.position;
        response.enc2pos = encoder1.position;
        
        response.enc1spd = motor1.getSpeedSigned();
        response.enc2spd = motor2.getSpeedSigned();
        
        response.enc1stat = motor1.getStatus();
        response.enc2stat = motor2.getStatus();
        
        response.isTurning = controlledMotors.isTurning();
        
        cmdMessenger.sendCmdBinArg<SensorResponse>(response);

        // Send the closing byte.
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
    // This is the list of recognized commands.
    // These can be commands that can either be sent or received.
    // In order to receive, attach a callback function to these events.
    // This enum must be the same length and order as the 'commands' list
    // in the Python client code.
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
