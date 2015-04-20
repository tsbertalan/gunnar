#ifndef GUNNAR_H
#define GUNNAR_H
#include <MemoryFree.h>
#include "Arduino.h"
#include "motorPIDcontrol.h"
#include "constants.h"
#include "encoders.h"
#include "vision.h"




// Consolidate as many globals as possible in a singleton robot.
class Gunnar
{
public:
    void init()
    {
        Serial.print(F("Free RAM: "));
        Serial.print(freeMemory());
        Serial.println(F(" bytes"));
        Serial.println(F("initializing Gunnar"));
        motor1 = Motor();
        motor1.init(MOTORLEFT);
        motor2 = Motor();
        motor2.init(MOTORRIGHT);
        bothMtrs[0] = &motor1;
        bothMtrs[1] = &motor2;
        
        encoder0.init(encoder0PinA, encoder0PinB, NULL);
        encoder1.init(encoder1PinA, encoder1PinB, NULL);
        Serial.println(F("Done initializing  encoders."));
        
        controlledMotors.init(&motor1, &motor2, &encoder0, &encoder1);
        
        // True black magic:
        sonarTask.init(this, &Gunnar::checkSonar, sonarPeriod);
        motorPIDsTask.init(this, &Gunnar::updatePIDs, PIDperiod);
        
        tasks[0] = &sonarTask;
        tasks[1] = &motorPIDsTask;
        
        taskDriver.init(ntasks, tasks);
        
        Serial.println(F("setup Gunnar"));
        pinMode(PIN_ACTIVITYSWITCH, INPUT);
        
        motor1.run(MOTORRELEASE);
        motor2.run(MOTORRELEASE);

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
    
    void decideTurn(int absAngle)
    {
        if(controlledMotors.isTurning())
        {
            int timeTurning = millisViaMicros() - _lastTurnTime;
            Serial.print(F("We've been turning for the last "));
            Serial.print(timeTurning);
            Serial.println(F(" ms."));
            if(timeTurning > maxTurnTime)
            {
                controlledMotors.stop();
            }
        }
        else
        {
            controlledMotors.stop();
            _lastTurnTime = millisViaMicros();
            
            const int delayBeforeMeasurement = 2;
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
            int angle;
            
            sensors.setPan(0);
            sensors.setTilt(0);
            
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

    void decideTurn()
    {
        decideTurn(-1);
    }
    
    Encoder encoder0;
    Encoder encoder1;
    
private:
    int _nTurns;
    int _lastTurnTime;
    
    Sensors sensors;
    ControlledMotors controlledMotors;

    Motor motor1;
    Motor motor2;
    Motor* bothMtrs[2];
    
    typedef void (Gunnar::* GunnarMemFn) ();
    
    class Task
    {
    public:
        void init(Gunnar* that, GunnarMemFn action, int frequency)
        {
            freq = frequency;
            _action = action;
            _that = that;
            lastExecution = micros() * 1000L;
            Serial.println("initializing task");
        };
        
        void execute()
        { 
            (_that->*_action)();
            lastExecution = micros() * 1000L;
        }
        
        long lastExecution;
        
        int freq;
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
        
        void run()
        {
            Serial.println("running taskDriver");
            Serial.print("Free memory: ");
            Serial.print(freeMemory());
            Serial.println(" bytes");
            
            while(1)
            {
                for(int i=0; i<_ntasks; i++)
                {
                    long now = micros()*1000L;
                    Task t = *_tasks[i];
                    if(now - t.lastExecution >= (long) t.freq)
                        t.execute();
                }
            }
        }
        
    private:
        Task** _tasks;
        int _ntasks;
    };

    TaskDriver taskDriver;
    Task sonarTask;
    Task motorPIDsTask;
    Task* tasks[ntasks];
    
};

#endif
