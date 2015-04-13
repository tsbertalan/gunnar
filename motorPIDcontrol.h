#ifndef MOTORPIDCONTROL_H
#define MOTORPIDCONTROL_H
#include <PID_v1.h>
#include "pinDefinitions.h"
#include "encoders.h"
#include "Arduino.h"


const double KP = 2.0;
const double KI = 1.5;
const double KD = .1;

double leftMotorControlledSpeed;
double leftMotorSetPoint;

double rightMotorControlledSpeed;
double rightMotorSetPoint;

void controlMotorSpeeds(double left, double right);

class ControlledMotors
{
public:
    ControlledMotors(Adafruit_DCMotor* leftMotor,
                     Adafruit_DCMotor* rightMotor,
                     Encoder encoder0,
                     Encoder encoder1)
    {
        // Setup stuff should go here also, if possible
        *leftPID = PID(&monitoredLeft,  &leftMotorControlledSpeed,  &leftMotorSetPoint,
                    KP, KI, KD, DIRECT);
        *rightPID = PID(&monitoredRight, &rightMotorControlledSpeed, &rightMotorSetPoint,
                     KP, KI, KD, DIRECT);
        leftPID->SetMode(AUTOMATIC);
        rightPID->SetMode(AUTOMATIC);
        
        setAccelLimit(MAXPWMSPEED);
        
        encoders[0] = &encoder0; // I don't know why an initializer list
        encoders[1] = &encoder1; // wasn't ok.
        pids[0] = leftPID;
        pids[1] = rightPID;
        setPoints[0] = &leftMotorSetPoint;
        setPoints[1] = &rightMotorSetPoint;
        ctrlVals[0] = &leftMotorControlledSpeed;
        ctrlVals[1] = &rightMotorControlledSpeed;
        monVals[0] = &monitoredLeft;
        monVals[1] = &monitoredRight;
        bothMtrs[0] = leftMotor;
        bothMtrs[1] = rightMotor;
    }
    
    void turn(int angle)
    {
        resetEncoders();
        Serial.print("Turning ");
        Serial.print(angle);
        Serial.println(" degrees.");
        angle = (double) angle * 120./90.; // 120 ticks/90 deg
        Serial.print("(");
        Serial.print(angle);
        Serial.println(" ticks)");
    //     long delay = 9.0*(double) angle/120.0 * 1000000L; // us
        long delay = 3L * 1000000L;
        uint8_t sgn;
        if(angle>0)
        {
            // Right turn. Right tread should go backwards.
            sgn = 1;
        }
        else
        {
            // Left turn. Right tread should go forwards.
            sgn = -1;
        }
        if(digitalRead(PIN_ACTIVITYSWITCH) == HIGH)
        {
            long startTime = micros();
            while(1)
            {
                controlMotorPositions(sgn*angle, -sgn*angle);
                if(micros() - startTime > delay)
                    break;
            }
        }
        else
        {
            stop();
        }
        }
        
    void stop()
    {
        int directions[] = {0, 0};
        for(uint8_t i=0; i<2; i++)
        {
            bothMtrs[i]->run(RELEASE);
            bothMtrs[i]->setSpeed(0);
        }
    }
    
    void updatePIDs()
    {
        // Update the PIDs
    //     Serial.println("micros  | m| setp  | monitr| ctrl| pos| updela");
                    //16376732, 0, 100.00, 157.00, 0.00, 157, 117132
                    //80056232, 1, 0.00, 158.00, 0.00, 168, 134332
        uint8_t i;
        for(i=0; i<2; i++)
        {
            *monVals[i] = encoders[i]->position;
            pids[i]->Compute();
            
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
            Serial.println(encoders[i]->trueUpdateDelay);

            // Apply the PID output.
            double ctrlVal =  *ctrlVals[i];
            if(abs(ctrlVal) < 15)
                ctrlVal = 0;
            if(ctrlVal == 0)
            {
                bothMtrs[i]->run(RELEASE);
            }
            else
            {
                if(ctrlVal < 0)
                {
                    bothMtrs[i]->run(BACKWARD);
                }
                else
                {
                    bothMtrs[i]->run(FORWARD);
                }
            }
            bothMtrs[i]->setSpeed(abs(ctrlVal));
        }
    }
    
    void setAccelLimit(int pwm)
    {
        leftPID->SetOutputLimits(-pwm, pwm);
        rightPID->SetOutputLimits(-pwm, pwm);
    }
    
    void go(int dist)
    {
        resetEncoders();
        Serial.print("Going ");
        Serial.print(dist);
        Serial.println(" cm.");
        dist = (double) dist * 100./23.; // 100 ticks/cm
        Serial.print("(");
        Serial.print(dist);
        Serial.println(" ticks)");
    //     long delay = 5.0*(double) dist/100.0 * 1000000L; // us
        long delay = 5L * 1000000L;
        if(digitalRead(PIN_ACTIVITYSWITCH) == HIGH)
        {
            long startTime = micros();
            while(1)
            {
                controlMotorPositions(dist, dist);
                if(micros() - startTime > delay)
                    break;
            }
        }
        else
        {
            stop();
        }
    }
private:
    void controlMotorPositions(long position0, long position1)
    {
        *setPoints[0] = (double) position0;
        *setPoints[1] = (double) position1;
    }
    void resetEncoders()
    {
        noInterrupts();
        encoders[0]->position = 0;
        encoders[1]->position = 0;
        interrupts();
    }
    int MAXPWMSPEED = 255;
    double monitoredLeft;
    double monitoredRight;
    Encoder* encoders[2];
    Adafruit_DCMotor* bothMtrs[2];
    PID* pids[2];
    double* setPoints[2];
    double* ctrlVals[2];
    double* monVals[2];
    PID* leftPID;
    PID* rightPID;
};
ControlledMotors* controlledMotors;
    
#endif
