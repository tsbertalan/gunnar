#include "motorPIDcontrol.h"
double monitoredLeft;
double monitoredRight;
PID leftPID(&monitoredLeft,  &leftMotorControlledSpeed,  &leftMotorSetPoint,  KP, KI, KD, DIRECT);
PID rightPID(&monitoredRight, &rightMotorControlledSpeed, &rightMotorSetPoint, KP, KI, KD, DIRECT);
PID pids[] = {leftPID, rightPID};

void motorPIDcontrolSetup()
{
    leftPID.SetMode(AUTOMATIC);
    rightPID.SetMode(AUTOMATIC);
    const int MAXPWMSPEED = 255;
    leftPID.SetOutputLimits(-MAXPWMSPEED, MAXPWMSPEED);
    rightPID.SetOutputLimits(-MAXPWMSPEED, MAXPWMSPEED);
}

void controlMotorPositions(double left, double right)
{
    double *setPoints[] = {&leftMotorSetPoint, &rightMotorSetPoint};
    double *ctrlVals[] = {&leftMotorControlledSpeed, &rightMotorControlledSpeed};
    double *monVals[] = {&monitoredLeft, &monitoredRight};
    PID *pids[] = {&leftPID, &rightPID};
    
    Encoder *encoders[] = {&encoder0, &encoder1};
    
    // Update the PIDs
    leftMotorSetPoint = left;
    rightMotorSetPoint = right;
    
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
