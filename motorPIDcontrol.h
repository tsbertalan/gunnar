#ifndef MOTORPIDCONTROL_H
#define MOTORPIDCONTROL_H
#include "pids.h"
#include "constants.h"
#include "encoders.h"
#include "Arduino.h"
#include "motors.h"

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
    
    void turn(int angle)
    {
        _resetEncoders();
        Serial.print("Turning ");
        Serial.print(angle);
        Serial.println(" degrees.");
        angle = (double) angle * 120./90.; // 120 ticks/90 deg
        Serial.print("(");
        Serial.print(angle);
        Serial.println(" ticks)");
    //     long delay = 9.0*(double) angle/120.0 * 1000000L; // us
        unsigned long delay = 3L * 1000000L;
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
                _controlMotorPositions(sgn*angle, -sgn*angle);
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
        motorStop();
    }
    
    void updatePIDs()
    {
        // Update the PIDs
        uint8_t i;
        for(i=0; i<2; i++)
        {
            *monVals[i] = (double) encoders[i]->position;
            pids[i].Compute();
            
//             if(i==0)
//             {
// Serial.println("micros  | m| setp  | monitr| ctrl   | pos| updela");
              //25492644, 0, -43.00, 364.00, -255.00, 369, 8136
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
//                 Serial.print(pids[i].outMax);
//                 Serial.print(", ");
                Serial.println(encoders[i]->trueUpdateDelay);
//             }

            // Apply the PID output.
//             if(abs(*ctrlVals[i]) < 15 || *ctrlVals[i] == 0)
//             {
//                 bothMtrs[i]->run(MOTORRELEASE);
//             }
//             else
//             {
//                 if(*ctrlVals[i] < 0)
//                 {
//                     bothMtrs[i]->run(MOTORBACKWARD);
//                 }
//                 else
//                 {
//                     bothMtrs[i]->run(MOTORFORWARD);
//                 }
//             }
//             else
//             {
                bothMtrs[i]->setSpeed(*ctrlVals[i]);
//             }
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
        _resetEncoders();
        Serial.print("Going ");
        Serial.print(dist);
        Serial.println(" cm.");
        dist = (double) dist * 100./23.; // 100 ticks / 23 cm
        Serial.print("(");
        Serial.print(dist);
        Serial.println(" ticks)");
    //     long delay = 5.0*(double) dist/100.0 * 1000000L; // us
        long startTime = micros();
        while(1)
        {
            unsigned long delay = GOSECONDS * 1000000L;  // Arbitrary delay.
            if(digitalRead(PIN_ACTIVITYSWITCH) == HIGH)
            {
                _controlMotorPositions(dist, dist);
                if(micros() - startTime > delay)
                    break;
            }
            else
            {
                stop();
            }
        }
    }
    
private:
    double leftMotorControlledSpeed;
    double rightMotorControlledSpeed;
    
    double leftMotorSetPoint;
    double rightMotorSetPoint;
    
    void _controlMotorPositions(long position0, long position1)
    {
        *setPoints[0] = (double) position0;
        *setPoints[1] = (double) position1;
        updatePIDs();  // TODO: Mabye updatePIDs() should be private?
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
