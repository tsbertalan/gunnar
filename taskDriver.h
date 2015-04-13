#ifndef TASKDRIVER_H
#define TASKDRIVER_H
#include "Arduino.h"
#include "vision.h"
#include "motorPIDcontrol.h"
#include "encoders.h"
#include "pinDefinitions.h"

class Task
{
public:
    Task(void (*action)(), int frequency)
    {
        freq = frequency;
        _action = action;
        lastExecution = micros() * 1000L;
    };
    
    void execute()
    { 
        _action();
        lastExecution = micros() * 1000L;
    }
    
    long lastExecution;
    
    int freq;
private:
    void (* _action)();
};

class TaskDriver
{
public:
    TaskDriver(const int ntasks, Task *taskArr[])
    {
        
        _tasks = taskArr;
        _ntasks = ntasks;
    };
    
    void run()
    {
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

void checkSonar();
Task sonar = Task(checkSonar, 100); // Check sonar every .1 seconds.
void updatePIDs()
{
    controlledMotors->updatePIDs();
}
Task motorPIDs = Task(updatePIDs, 32); // Update motor PIDs every .032 seconds.

const int ntasks = 2;
Task* tasks[ntasks];
TaskDriver *taskDriverp;
extern TaskDriver taskDriver;

void taskerSetup();
void taskerLoop();
#endif