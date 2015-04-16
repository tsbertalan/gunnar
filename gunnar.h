#ifndef GUNNAR_H
#define GUNNAR_H
#include "Arduino.h"
#include "motorPIDcontrol.h"
#include "constants.h"
#include "encoders.h"
#include "motorPIDcontrol.h"
#include "Adafruit_MotorShield_modified.h"
#include "vision.h"




// Consolidate as many globals as possible in a singleton robot.

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

class Gunnar
{
public:
    void init()
    {
        Serial.println("initializing Gunnar");
        motor1 = AFMS.getMotor(1);
        motor2 = AFMS.getMotor(2);
        bothMtrs[0] = motor1;
        bothMtrs[1] = motor2;
        
        encoder0 = Encoder();
        encoder0.init(encoder0PinA, encoder0PinB, motor1);
        encoder1 = Encoder();
        encoder1.init(encoder1PinA, encoder1PinB, motor2);
        Serial.println("Done initializing  encoders.");
        controlledMotors = ControlledMotors();
        controlledMotors.init(motor1, motor2, encoder0, encoder1);
        
        // True black magic:
        sonarTask.init(this, &Gunnar::checkSonar, 100);
        motorPIDsTask.init(this, &Gunnar::updatePIDs, 100);
        
        tasks[0] = &sonarTask;
        tasks[1] = &motorPIDsTask;
        
        taskDriver.init(ntasks, tasks);
        
        sensors.init();
    }
    
    void setup()
    {
        Serial.println("setup Gunnar");
        pinMode(PIN_ACTIVITYSWITCH, INPUT);
        AFMS.begin(1600);
        
        motor1->run(RELEASE);
        motor2->run(RELEASE);

        sensors.setup();
    }
    
    void loop()
    {
        taskDriver.run();
    }

    int nTurns;
    void checkSonar()
    {
        float dist = sensors.getSonarDist(8);
        Serial.print("sighted distance: "); Serial.println(dist);
        if(dist < 60)
        {
            if(dist < minimumSensableDistance)
            {
                controlledMotors.stop();
                controlledMotors.go(-7);
            }
            else // We're not *super* close.
            {
                if(nTurns > 4)
                {
                    nTurns = 0;
                    controlledMotors.stop();
                    controlledMotors.go(-7);
                }
                else // We haven't turned very many times.
                {
                    Serial.println("Obstacle sighted. Turning.");
                    controlledMotors.stop();
                    decideTurn();
                    nTurns++;
                }
            }
        }
        else // dist >= 60
        {
            nTurns = 0;
            controlledMotors.go(dist/4);
        }
    }
    
    void updatePIDs()
    {
        controlledMotors.updatePIDs();
    }
    
    void decideTurn(int absAngle)
    {
        const int delayBeforeMeasurement = 16;
        const int nMeasurements = 8;
        sensors.setTilt(10); // Look up slightly.
        
        sensors.setTilt(-45);
        interruptibleDelay(delayBeforeMeasurement);
        float rdist = sensors.getSonarDist(nMeasurements);
    //     Serial.print("rdist: ");
    //     Serial.println(rdist);
        
        sensors.setTilt(45);
        interruptibleDelay(delayBeforeMeasurement);
        float ldist = sensors.getSonarDist(nMeasurements);
    //     Serial.print("ldist: ");
    //     Serial.println(ldist);
        int angle;
        
        sensors.setTilt(0);
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
        
    //     interruptibleDelay(500);
        
        interruptibleDelay(150);
        sensors.disableServos();
        
        controlledMotors.turn(angle);
    }

    void decideTurn()
    {
        decideTurn(-1);
    }
    
    Encoder encoder0;
    Encoder encoder1;
    
private:
    Sensors sensors;
    ControlledMotors controlledMotors;

    Adafruit_DCMotor* motor1;
    Adafruit_DCMotor* motor2;
    Adafruit_DCMotor* bothMtrs[2];
    
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

Gunnar gunnar;

#endif