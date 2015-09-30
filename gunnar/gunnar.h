#ifndef GUNNAR_H
#define GUNNAR_H
#include "Arduino.h"
#include "motorPIDcontrol.h"
#include "constants.h"
#include "encoders.h"
#include "vision.h"

class Gunnar;
typedef void (Gunnar::* GunnarMemFn) ();


class Task
{
public:
    void init(Gunnar* that, GunnarMemFn action, int frequency, boolean initializeAsActive=true)
    {
        active = initializeAsActive;
        freq = frequency;
        _action = action;
        _that = that;
        lastExecution = (long) micros() / 1000L;
        Serial.println("initializing task");
    };
    
    void execute(long now)
    { 
        (_that->*_action)();
        lastExecution = now;
    }
    
    long lastExecution;
    int freq;
    boolean active;
    
private:
    Gunnar* _that;
    GunnarMemFn _action;
};


class TaskDriver
{
public:
    void init(const int ntasks, Task *taskArr[])
    {
        Serial.println("initializing taskDriver");   
        _tasks = taskArr;
        _ntasks = ntasks;
    };
    
    void run(long duration=0)
    {
        // Duration in microseconds. 0 means endless.
        Serial.println("running taskDriver");
        startTime = (long) micros();
        
        while(true)
        {
            for(int i=0; i<_ntasks; i++)
            {
                long now = (long) micros() / 1000L;  // ms
                Task *t = _tasks[i];
                if(t->active and (now - t->lastExecution > t->freq))
                {
                    t->execute(now);
                }
            }
            
            if((duration > 0) and ((long) micros() - startTime > duration))
                break;
        }
    }
    
private:
    Task** _tasks;
    int _ntasks;
    long startTime;
};


// Consolidate as many globals as possible in a singleton robot.
class Gunnar
{
public:
    void init()
    {
        Serial.println(F("initializing Gunnar"));
        for(uint8_t i=0; i<3; i++)
        {
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
        Serial.println(F("Done initializing  encoders."));
        
        controlledMotors.init(&motor1, &motor2, &encoder0, &encoder1, &sensors);
        
        // True black magic:
        sonarTask.init(this, &Gunnar::checkSonar, sonarPeriod);
        motorPIDsTask.init(this, &Gunnar::updatePIDs, PIDperiod);
        ahrsUpdateTask.init(this, &Gunnar::updateAHRS, ahrsPeriod);
        sendDataTask.init(this, &Gunnar::sendData, sendDataPeriod);
                
        tasks[0] = &sonarTask;
        tasks[1] = &motorPIDsTask;
        tasks[2] = &ahrsUpdateTask;
        tasks[3] = &sendDataTask;
        
        taskDriver.init(ntasks, tasks);
        
        Serial.println(F("setup Gunnar"));
        pinMode(PIN_ACTIVITYSWITCH, INPUT);
        
        motor1.stop();
        motor2.stop();

        sensors.init();
    }
    
    void loop()
    {
        taskDriver.run();
    }
    
    void testBackForth()
    {
        controlledMotors.go(10);
        controlledMotors.stop();
        controlledMotors.go(-10);
        controlledMotors.go(256);
        controlledMotors.go(-128);
        controlledMotors.stop();
        interruptibleDelay(1000);
    }
    
    void ramp(int a, int b)
    {
        if(a < b)
        {
            for(int j=a; j<b; j++)
            {
                Serial.print("j=");
                Serial.println(j);
                for(boolean i=0; i<2; i++)
                {
                    bothMtrs[i]->setSpeed(j);
                }
                interruptibleDelay(10);
            }
        }
        else
        {
            for(int j=a; j>b; j--)
            {
                Serial.print("j=");
                Serial.println(j);
                for(boolean i=0; i<2; i++)
                {
                    bothMtrs[i]->setSpeed(j);
                }
                interruptibleDelay(10);
            }
        }
    }
    
    void testSpeedSweep()
    {
        ramp(0, 128);
        ramp(128, -128);
        ramp(-128, 0);
    }
    
    void checkSonar()
    {
        if(checkActivitySwitch())
        {
            const int backoff = 30;
            float dist = sensors.getSonarDist(8);
            Serial.print(F("sighted distance: ")); Serial.println(dist);
            if(dist < turnThresh)
            {
                if(dist < minimumSensableDistance)
                {
                    controlledMotors.stop();
                    controlledMotors.go(-backoff);
                }
                else // We're not *super* close.
                {
                    if(_nTurns > 4)
                    {
                        _nTurns = 0;
                        controlledMotors.stop();
                        controlledMotors.go(-backoff);
                    }
                    else // We haven't turned very many times.
                    {
                        Serial.println("Obstacle sighted. Turning.");
                        decideTurn();
                        _nTurns++;
                    }
                }
            }
            else // dist >= 60
            {
                _nTurns = 0;
                controlledMotors.go(dist/4);
            }
        }
        else
        {
            sensors.disableServos();
            controlledMotors.stop();
        }
    }
    
    void updatePIDs()
    {
        controlledMotors.updatePIDs();
    }
    
    void updateAHRS()
    {
        sensors.ahrs.update();
    }
    
    void decideTurn(int absAngle=-1)
    {
        // If We're currently in the process of turning,
        // only initiate a new turn if the current turn error has gone below
        // threshold.
        int timeTurning = millisViaMicros() - _lastTurnTime;
        boolean doTurn = true;
        if(controlledMotors.isTurning())
        {
            doTurn = false;
            
            Serial.print(F("We've been turning for the last "));
            Serial.print(timeTurning);
            Serial.println(F(" ms."));
            if(timeTurning > maxTurnTime)
            {
                Serial.println(F("Turning timeout."));
                doTurn = true;
            }
            
            if(controlledMotors.getEucError() < EUCCONTROLERRORTHRESH)
            {
                Serial.println(F("Turning goal reached. New turn innitiated."));
            }
        }
        
        if(doTurn)
        {
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
            
            // Arduino's sprintf doesn't have %f.
            // stackoverflow.com/questions/27651012
            char msg[64];
            char rtmp[8];
            char ltmp[8];
            dtostrf(rdist, 6, 2, rtmp);
            dtostrf(ldist, 6, 2, ltmp);
            
            if( absAngle == -1)
            {
                float C = sqrt(ldist*ldist + rdist*rdist);
                absAngle = 180.0 / 3.14159 * asin(min(ldist, rdist) / C);
            }
            absAngle = abs(absAngle);
            
            int angle;
            if(max(rdist, ldist) < minimumSensableDistance)
            {
                Serial.println("Too close to wall to decide which way to turn.");
                angle = 180;
            }
            else
            {
                if( rdist < ldist )
                {
                    
                    sprintf(msg, "rdist=%s < ldist=%s -- ", rtmp, ltmp);
                    Serial.print(msg);
                    angle = absAngle;
                }
                else
                {
                    sprintf(msg, "rdist=%s >= ldist=%s -- ", rtmp, ltmp);
                    Serial.print(msg);
                    angle = -absAngle;
                }
            }
                
            Serial.print("angle=");
            Serial.println(angle);

            sensors.disableServos();
            
            controlledMotors.turn(angle);
        }
    }

    void sendData()
    {
        Serial.print("#"); Serial.print(sensors.getSonarDist());
        
        Serial.print("#"); Serial.print(sensors.ahrs.getHeading());
        Serial.print("#"); Serial.print(sensors.ahrs.orientation.heading);
        Serial.print("#"); Serial.print(sensors.ahrs.orientation.roll);
        Serial.print("#"); Serial.print(sensors.ahrs.orientation.pitch);
        
        Serial.print("#"); Serial.print(sensors.ahrs.orientation.x);
        Serial.print("#"); Serial.print(sensors.ahrs.orientation.y);
        Serial.print("#"); Serial.print(sensors.ahrs.orientation.z);
        
        Serial.print("#"); Serial.print(sensors.ahrs.orientation.v[0]);
        Serial.print("#"); Serial.print(sensors.ahrs.orientation.v[1]);
        Serial.print("#"); Serial.print(sensors.ahrs.orientation.v[2]);
        
        Serial.print("#"); Serial.print(encoder0.position);
        Serial.print("#"); Serial.print(motor1.getSpeed());
        Serial.print("#"); Serial.print(motor1.getStatus());
        
        Serial.print("#"); Serial.print(encoder1.position);
        Serial.print("#"); Serial.print(motor2.getSpeed());
        Serial.print("#"); Serial.print(motor2.getStatus());
        
        Serial.print("#"); Serial.print(controlledMotors.isTurning());
        
        Serial.println("");
    }
    
    Encoder encoder0;
    Encoder encoder1;
    Sensors sensors;
    ControlledMotors controlledMotors;
    Motor motor1;
    Motor motor2;
    
    TaskDriver taskDriver;
    Task sonarTask;
    Task motorPIDsTask;
    Task ahrsUpdateTask;
    Task sendDataTask;
    
private:
    int _nTurns;
    int _lastTurnTime;

    Motor* bothMtrs[2];
    
    Task* tasks[ntasks];
    
};


#endif
