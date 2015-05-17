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
    void init(
                Motor* leftMotor,
                Motor* rightMotor,
                Encoder *encoder0,
                Encoder *encoder1,
                Sensors *sensorsp
             )
    {
        sensors = sensorsp;
        Serial.println("initializing motors");
        leftPID.init(&monitoredLeft,  &leftMotorControlledSpeed,  &leftMotorSetPoint,
                    KP, KI, KD, DIRECT);
        rightPID.init(&monitoredRight, &rightMotorControlledSpeed, &rightMotorSetPoint,
                     KP, KI, KD, DIRECT);
        leftPID.SetMode(AUTOMATIC);
        rightPID.SetMode(AUTOMATIC);
        
        anglePID.init(&monitoredAngle, &angleCtrlVal, &angleSetPoint,
                      KP, KI, KD, DIRECT);
        anglePID.SetMode(AUTOMATIC);
        
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
    
    class Signal
    {
    public:
        void none()
        {
            analogWrite(TURNSIGNALLEFTPIN, 0);
            analogWrite(TURNSIGNALRIGHTPIN, 0);
            analogWrite(TURNSIGNALBACKPIN, 0);
            analogWrite(TURNSIGNALFORWARDPIN, 0);
        }
        
        void right()
        {
            analogWrite(TURNSIGNALLEFTPIN, 0);
            analogWrite(TURNSIGNALRIGHTPIN, 255);
            analogWrite(TURNSIGNALBACKPIN, 0);
            analogWrite(TURNSIGNALFORWARDPIN, 0);
        }
        
        void left()
        {
            analogWrite(TURNSIGNALLEFTPIN, 255);
            analogWrite(TURNSIGNALRIGHTPIN, 0);
            analogWrite(TURNSIGNALBACKPIN, 0);
            analogWrite(TURNSIGNALFORWARDPIN, 0);
        }
        
        void stop()
        {
            analogWrite(TURNSIGNALLEFTPIN, 255);
            analogWrite(TURNSIGNALRIGHTPIN, 255);
            analogWrite(TURNSIGNALBACKPIN, 0);
            analogWrite(TURNSIGNALFORWARDPIN, 0);
        }
        
        void forward()
        {
            analogWrite(TURNSIGNALLEFTPIN, 0);
            analogWrite(TURNSIGNALRIGHTPIN, 0);
            analogWrite(TURNSIGNALBACKPIN, 0);
            analogWrite(TURNSIGNALFORWARDPIN, 128);
        }
        
        void backward()
        {
            analogWrite(TURNSIGNALLEFTPIN, 0);
            analogWrite(TURNSIGNALRIGHTPIN, 0);
            analogWrite(TURNSIGNALBACKPIN, 255);
            analogWrite(TURNSIGNALFORWARDPIN, 0);
        }
    } signal;
    
    boolean isTurning()
    {
        return _turning;
    }
    
    void turn(int angle)
    {
        _turning = true;
        if(digitalRead(PIN_ACTIVITYSWITCH) == HIGH)
        {
            double currHeading = sensors->ahrs.getHeading();
            boolean rightTurn = angle > 0;
            _resetEncoders();
            Serial.print("Turning ");
            Serial.print(angle);
            Serial.println(" degrees.");
            
            double newHeading = fmod(
                    180.0 + currHeading + (double) angle,
                    360.0
                ) - 180.0;
                
            Serial.print("(To heading ");
            Serial.print((int) newHeading);
            Serial.print(" from heading ");
            Serial.print((int) currHeading);
            Serial.println(".)");
            
            if(rightTurn)
            {
                // Right turn. Right tread should go backwards.
                signal.right();
            }
            else
            {
                // Left turn. Right tread should go forwards.
                signal.left();
            }
            controlAngle(newHeading);
        }
        else
        {
            stop();
        }
    }
        
    void stop()
    {
        _turning = false;
        signal.stop();
        bothMtrs[0]->stop();
        bothMtrs[1]->stop();
    }
    
    void printCtrlStatus()
    {
    #define PRINTCTRLDATA
    #ifdef PRINTCTRLDATA
        for(uint8_t i=0; i<2; i++)
        {
//             if(i==0)
//             {
    //     Serial.println("micros  | m| setp | moni | ctrl  | ps| spd  | updela");
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
    #endif
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
                signal.backward();
            }
            else
            {
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
        }
        else
        {
            stop();
        }
    }
    
    double getEucError()
    {
        double err0 = *setPoints[0] - *monVals[0];
        double err1 = *setPoints[1] - *monVals[1];
        return sqrt(err0*err0 + err1*err1);
    }
    
    void updatePIDs()
    {
        if(checkActivitySwitch())
        {
            uint8_t i;
            updateMonitoredValues();
            if(_turning)
            {
                anglePID.Compute();
                bothMtrs[0]->setSpeed(angleCtrlVal);
                bothMtrs[1]->setSpeed(-angleCtrlVal);
            }
            else
            {
                // Update the PIDs
                for(i=0; i<2; i++)
                {
                    pids[i].Compute();
                    bothMtrs[i]->setSpeed(*ctrlVals[i]);
                }
                printCtrlStatus();
            }
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
        if(_turning)
        {
            monitoredAngle = sensors->ahrs.getHeading();
        }
        else
        {
            for(uint8_t i=0; i<2; i++)
            {
                *monVals[i] = (double) encoders[i]->position;
            }
        }
    }
    
    void _controlMotorPositions(double position0, double position1)
    {
        *setPoints[0] = (double) position0;
        *setPoints[1] = (double) position1;
        updateMonitoredValues();
        updatePIDs();
    }
    
    void controlAngle(double angle)
    {
        angleSetPoint = angle;
        sensors->ahrs.update();
        updateMonitoredValues();
        updatePIDs();
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
    Sensors *sensors;
    PID pids[2];
    double monitoredLeft;
    double monitoredRight;
    double* setPoints[2];
    double* ctrlVals[2];
    double* monVals[2];
    PID leftPID;
    PID rightPID;
    
    double monitoredAngle;
    double angleSetPoint;
    double angleCtrlVal;
    PID anglePID;
};


#endif
