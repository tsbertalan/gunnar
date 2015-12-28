#ifndef MOTORPIDCONTROL_H
#define MOTORPIDCONTROL_H
#include "pids.h"
#include "constants.h"
#include "encoders.h"
#include "Arduino.h"
#include "motors.h"
#include "sensors.h"

class ControlledMotors {
public:
    void init(
        Motor* leftMotor,
        Motor* rightMotor,
        Encoder *encoder0,
        Encoder *encoder1,
        Sensors *sensorsp
    ) {
        pinMode(encoder0PinA, INPUT);
        pinMode(encoder0PinB, INPUT);
        fakeAngleSetPoint = 0;
        sensors = sensorsp;
        leftPID.init(&monitoredLeft,  &leftMotorControlledSpeed,  &leftMotorSetPoint,
                     KP, KI, KD, DIRECT);
        rightPID.init(&monitoredRight, &rightMotorControlledSpeed, &rightMotorSetPoint,
                      5.0, KI, KD, DIRECT);
        anglePID.init(&angleError, &angleCtrlVal, &fakeAngleSetPoint,
                      KP, KI, KD, DIRECT);

        leftPID.SetMode(AUTOMATIC);
        rightPID.SetMode(AUTOMATIC);
        anglePID.SetMode(AUTOMATIC);

        leftPID.SetSampleTime(PIDperiod);
        rightPID.SetSampleTime(PIDperiod);
        anglePID.SetSampleTime(PIDperiod);


        encoders[0] = encoder0; // We don't use an initializer list
        encoders[1] = encoder1; // because we can't control when global objects.
        pids[0] = leftPID;      // are constructed, and controlledMotors is
        pids[1] = rightPID;     // and its properties are owned by the global gunnar.
        setPoints[0] = &leftMotorSetPoint;
        setPoints[1] = &rightMotorSetPoint;
        ctrlVals[0] = &leftMotorControlledSpeed;
        ctrlVals[1] = &rightMotorControlledSpeed;
        monVals[0] = &monitoredLeft;
        monVals[1] = &monitoredRight;
        bothMtrs[0] = leftMotor;
        bothMtrs[1] = rightMotor;

        for(uint8_t i=0; i<2; i++) {
            pids[i].SetOutputLimits(-MAXPWMSPEED, MAXPWMSPEED);
        }
        anglePID.SetOutputLimits(-MAXPWMSPEED, MAXPWMSPEED);
    }

    class Signal {
    public:
        void none() {
            analogWrite(TURNSIGNALLEFTPIN, 0);
            analogWrite(TURNSIGNALRIGHTPIN, 0);
            analogWrite(TURNSIGNALBACKPIN, 0);
            analogWrite(TURNSIGNALFORWARDPIN, 0);
        }

        void right() {
            analogWrite(TURNSIGNALLEFTPIN, 0);
            analogWrite(TURNSIGNALRIGHTPIN, 255);
            analogWrite(TURNSIGNALBACKPIN, 0);
            analogWrite(TURNSIGNALFORWARDPIN, 0);
        }

        void left() {
            analogWrite(TURNSIGNALLEFTPIN, 255);
            analogWrite(TURNSIGNALRIGHTPIN, 0);
            analogWrite(TURNSIGNALBACKPIN, 0);
            analogWrite(TURNSIGNALFORWARDPIN, 0);
        }

        void stop() {
            analogWrite(TURNSIGNALLEFTPIN, 255);
            analogWrite(TURNSIGNALRIGHTPIN, 255);
            analogWrite(TURNSIGNALBACKPIN, 0);
            analogWrite(TURNSIGNALFORWARDPIN, 0);
        }

        void forward() {
            analogWrite(TURNSIGNALLEFTPIN, 0);
            analogWrite(TURNSIGNALRIGHTPIN, 0);
            analogWrite(TURNSIGNALBACKPIN, 0);
            analogWrite(TURNSIGNALFORWARDPIN, 128);
        }

        void backward() {
            analogWrite(TURNSIGNALLEFTPIN, 0);
            analogWrite(TURNSIGNALRIGHTPIN, 0);
            analogWrite(TURNSIGNALBACKPIN, 255);
            analogWrite(TURNSIGNALFORWARDPIN, 0);
        }
    } signal;

    boolean isTurning() {
        return _turning;
    }

    void turn(int angle) {
        _turning = true;

        if(keyboardControl || checkActivitySwitch()) {

            double currHeading = sensors->ahrs.getHeading();
            boolean rightTurn = angle > 0;
            _resetEncoders();

            double newHeading = fmod(
                                    180.0 + currHeading + (double) angle,
                                    360.0
                                ) - 180.0;


            if(rightTurn) {
                // Right turn. Right tread should go backwards.
                signal.right();
            } else {
                // Left turn. Right tread should go forwards.
                signal.left();
            }
            controlAngle(newHeading);

        } else {
            stop();
        }
    }

    void stop() {
        _turning = false;
        signal.stop();
        bothMtrs[0]->stop();
        bothMtrs[1]->stop();
    }

    void printCtrlStatus() {
#define PRINTCTRLDATA
#ifdef PRINTCTRLDATA
        if(isTurning()) {
//             Serial.println("micros  | m| setp | moni | ctrl  | ps| spd  | updela");
            Serial.print(micros());
            Serial.print(", ");
            Serial.print(3);
            Serial.print(", ");
            Serial.print(angleSetPoint);
            Serial.print(", ");
            Serial.print(angleError);
            Serial.print(", ");
            Serial.print(angleCtrlVal);
            Serial.println(", ");
        } else {
            for(uint8_t i=0; i<2; i++) {
                //             if(i==0)
                //             {
//             Serial.println("micros  | m| setp | moni | ctrl  | ps| spd  | updela");
                //10575920, 1, 82.00, 12.00, 255.00, 15, 92.89, 17588
                Serial.print(micros());
                Serial.print(", ");
                Serial.print(i);
                Serial.print(", ");
                Serial.print(*setPoints[i]);
                Serial.print(", ");
                Serial.print(*monVals[i]);
                Serial.print(", ");
                Serial.print(*ctrlVals[i]);
                Serial.print(", ");
                Serial.print(encoders[i]->position);
                Serial.print(", ");
                Serial.print(encoders[i]->getSpeed());
                Serial.print(", ");
                Serial.println(encoders[i]->trueUpdateDelay);
                //             }
            }
        }
#endif
    }

    void go(int dist) {
        _turning = false;
        if(keyboardControl || checkActivitySwitch()) {
            if(dist < 0) {
                signal.backward();
            } else {
                signal.forward();
            }
            _resetEncoders();
            Serial.print("Going ");
            Serial.print(dist);
            Serial.println(" cm.");
            dist = (double) dist * 100./23.; // 100 ticks / 23 cm
            Serial.print("(");
            Serial.print(dist);
            Serial.println(" ticks)");
            _controlMotorPositions(dist, dist);
        } else {
            stop();
        }
    }

    double getEucError() {
        double err0 = *setPoints[0] - *monVals[0];
        double err1 = *setPoints[1] - *monVals[1];
        return sqrt(err0*err0 + err1*err1);
    }

    void updatePIDs() {
        if(keyboardControl || checkActivitySwitch()) {
            updateMonitoredValues();
//             printCtrlStatus();
            if(_turning) {
                anglePID.Compute();
                bothMtrs[0]->setSpeed(angleCtrlVal);
                bothMtrs[1]->setSpeed(-angleCtrlVal);
            } else {
                // Update the PIDs
                uint8_t i;
                for(i=0; i<2; i++) {
                    pids[i].Compute();
                    bothMtrs[i]->setSpeed(*ctrlVals[i]);
                }
            }
        } else {
            stop();
        }
    }

private:
    boolean _turning;
    double leftMotorControlledSpeed;
    double rightMotorControlledSpeed;

    double leftMotorSetPoint;
    double rightMotorSetPoint;

    double getAngleErr(double h, double g) {
        // Get the signed error between the heading h and the goal heading g.
        // Returned value should be in [-180, 180].
        // Assume -180 < h,g, < 180. Maybe <=.

        // Find the smaller arc between h and g.
        double delta;
        h>g ? delta=h-g : delta=g-h;
        while(delta > 180) {
            delta -= 180;
            delta *= -1;
            delta += 180;
        }
        // At this point, 0 < delta < 180 should be true.

        double gp = h + delta;
        while(gp > 180)
            gp -= 360;
        int m;
        gp==g ? m=1 : m=-1;
        return delta * m;
    }

    void updateMonitoredValues() {
        if(isTurning()) {
            angleError = getAngleErr(sensors->ahrs.getHeading(), angleSetPoint);
        } else {
            for(uint8_t i=0; i<2; i++) {
                *monVals[i] = (double) encoders[i]->position;
            }
        }
    }

    void _controlMotorPositions(double position0, double position1) {
        *setPoints[0] = (double) position0;
        *setPoints[1] = (double) position1;
        updateMonitoredValues();
        updatePIDs();
    }

    void controlAngle(double angle) {
        angleSetPoint = sensors->ahrs.getHeading() + angle;
        if(angleSetPoint > 180)
            angleSetPoint -= 360;
        if(angleSetPoint < -180)
            angleSetPoint += 360;
        updateMonitoredValues();
        updatePIDs();
    }

    void _resetEncoders() {
        noInterrupts();
        encoders[0]->position = 0;
        encoders[1]->position = 0;
        interrupts();
    }

    Motor *bothMtrs[2];
    Encoder *encoders[2];
    Sensors *sensors;
    PID pids[2];
    double monitoredLeft;
    double monitoredRight;
    double* ctrlVals[2];
    double* setPoints[2];
    double* monVals[2];
    PID leftPID;
    PID rightPID;

    double angleError;
    double angleCtrlVal;
    double fakeAngleSetPoint;
    double angleSetPoint;
    PID anglePID;
};


#endif
