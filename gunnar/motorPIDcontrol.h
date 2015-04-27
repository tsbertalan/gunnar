#ifndef MOTORPIDCONTROL_H
#define MOTORPIDCONTROL_H
#include "pids.h"
#include "constants.h"
#include "encoders.h"
#include "Arduino.h"
#include "motors.h"
#include "vision.h"

class ControlledMotors
{
public:
    void init(Motor* leftMotor,
              Motor* rightMotor,
              Encoder *encoder0,
              Encoder *encoder1)
    {
        Serial.println("initializing motors");
        leftPID.init(&monitoredLeft,  &leftMotorControlledSpeed,  &leftMotorSetPoint,
                    KP, KI, KD, DIRECT);
        rightPID.init(&monitoredRight, &rightMotorControlledSpeed, &rightMotorSetPoint,
                     KP, KI, KD, DIRECT);
        leftPID.SetMode(AUTOMATIC);
        rightPID.SetMode(AUTOMATIC);
        
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
        
        setAccelLimit(MAXPWMSPEED);
    }
    
    void setup()
    {
        pinMode(encoder0PinA, INPUT);
        pinMode(encoder0PinB, INPUT);
    }
    
    void signalRight()
    {
        analogWrite(TURNSIGNALLEFTPIN, 0);
        analogWrite(TURNSIGNALRIGHTPIN, 255);
        analogWrite(TURNSIGNALBACKPIN, 0);
        analogWrite(TURNSIGNALFORWARDPIN, 0);
    }
    
    void signalLeft()
    {
        analogWrite(TURNSIGNALLEFTPIN, 255);
        analogWrite(TURNSIGNALRIGHTPIN, 0);
        analogWrite(TURNSIGNALBACKPIN, 0);
        analogWrite(TURNSIGNALFORWARDPIN, 0);
    }
    
    void signalStop()
    {
        analogWrite(TURNSIGNALLEFTPIN, 255);
        analogWrite(TURNSIGNALRIGHTPIN, 255);
        analogWrite(TURNSIGNALBACKPIN, 0);
        analogWrite(TURNSIGNALFORWARDPIN, 0);
    }
    
    void signalForward()
    {
        analogWrite(TURNSIGNALLEFTPIN, 0);
        analogWrite(TURNSIGNALRIGHTPIN, 0);
        analogWrite(TURNSIGNALBACKPIN, 0);
        analogWrite(TURNSIGNALFORWARDPIN, 128);
    }
    
    void signalReverse()
    {
        analogWrite(TURNSIGNALLEFTPIN, 0);
        analogWrite(TURNSIGNALRIGHTPIN, 0);
        analogWrite(TURNSIGNALBACKPIN, 255);
        analogWrite(TURNSIGNALFORWARDPIN, 0);
    }
    
    boolean isTurning()
    {
        return _turning;
    }
    
    void turn(int angle)
    {
        _turning = true;
        if(digitalRead(PIN_ACTIVITYSWITCH) == HIGH)
        {
            _resetEncoders();
            Serial.print("Turning ");
            Serial.print(angle);
            Serial.println(" degrees.");
            angle = (double) angle * 120./90.; // 120 ticks/90 deg
            Serial.print("(");
            Serial.print(angle);
            Serial.println(" ticks)");
            uint8_t sgn;
            if(angle>0)
            {
                // Right turn. Right tread should go backwards.
                sgn = 1;
                signalRight();
            }
            else
            {
                // Left turn. Right tread should go forwards.
                sgn = -1;
                signalLeft();
            }
            _controlMotorPositions(sgn*angle, -sgn*angle);
        }
        else
        {
            stop();
        }
    }
        
    void stop()
    {
        _turning = false;
        signalStop();
        bothMtrs[0]->stop();
        bothMtrs[1]->stop();
    }
    
    void updatePIDs()
    {
        updateMonitoredValues();
        if(checkActivitySwitch())
        {
            
            // Update the PIDs
            uint8_t i;
            for(i=0; i<2; i++)
            {
                pids[i].Compute();
                
    //             if(i==0)
    //             {
    #ifdef PRINTCTRLDATA
    Serial.println("micros  | m| setp | moni | ctrl  | ps| spd  | updela");
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
    #endif

                bothMtrs[i]->setSpeed(*ctrlVals[i]);
            }
        }
        else
        {
            stop();
        }
    }
    
    void setAccelLimit(int pwm)
    {
        for(uint8_t i=0; i<2; i++)
        {
            pids[i].SetOutputLimits(-pwm, pwm);
        }
    }
    
    void go(int dist)
    {
        _turning = false;
        if(digitalRead(PIN_ACTIVITYSWITCH) == HIGH)
        {
            if(dist < 0)
            {
                signalReverse();
            }
            else
            {
                signalForward();
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
        }
        else
        {
            stop();
        }
    }
    
private:
    boolean _turning;
    double leftMotorControlledSpeed;
    double rightMotorControlledSpeed;
    
    double leftMotorSetPoint;
    double rightMotorSetPoint;
    
    void updateMonitoredValues()
    {
        for(uint8_t i=0; i<2; i++)
        {
            *monVals[i] = (double) encoders[i]->position;
        }
    }
    
    void _controlMotorPositions(double position0, double position1)
    {
        *setPoints[0] = (double) position0;
        *setPoints[1] = (double) position1;
        updateMonitoredValues();
        updatePIDs();  // TODO: Mabye updatePIDs() should be private?
    }
    
    double getEucError()
    {
        double err0 = *setPoints[0] - *monVals[0];
        double err1 = *setPoints[1] - *monVals[1];
        return sqrt(err0*err0 + err1*err1);
    }
    
    void _resetEncoders()
    {
        noInterrupts();
        encoders[0]->position = 0;
        encoders[1]->position = 0;
        interrupts();
    }
   
    Motor *bothMtrs[2];
    Encoder *encoders[2];
    PID pids[2];
    double monitoredLeft;
    double monitoredRight;
    double* setPoints[2];
    double* ctrlVals[2];
    double* monVals[2];
    PID leftPID;
    PID rightPID;
};


#endif
