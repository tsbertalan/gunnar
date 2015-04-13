#include "taskDriver.h"




void decideTurn(int absAngle)
{
    const int delayBeforeMeasurement = 16;
    const int nMeasurements = 8;
    setTilt(10); // Look up slightly.
    
    setPan(-45);
    interruptibleDelay(delayBeforeMeasurement);
    float rdist = getSonarDist(nMeasurements);
//     Serial.print("rdist: ");
//     Serial.println(rdist);
    
    setPan(45);
    interruptibleDelay(delayBeforeMeasurement);
    float ldist = getSonarDist(nMeasurements);
//     Serial.print("ldist: ");
//     Serial.println(ldist);
    int angle;
    
    setPan(0);
    setTilt(0);
    
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
    servoPan.detach();
    servoTilt.detach();
    
    controlledMotors->turn(angle);
}

void decideTurn()
{
    decideTurn(-1);
}




int nTurns;
void checkSonar()
{
    float dist = getSonarDist(8);
    Serial.print("sighted distance: "); Serial.println(dist);
    if(dist < 60)
    {
        if(dist < minimumSensableDistance)
        {
            controlledMotors->stop();
            controlledMotors->go(-7);
        }
        else // We're not *super* close.
        {
            if(nTurns > 4)
            {
                nTurns = 0;
                controlledMotors->stop();
                controlledMotors->go(-7);
            }
            else // We haven't turned very many times.
            {
                Serial.println("Obstacle sighted. Turning.");
                controlledMotors->stop();
                decideTurn();
                nTurns++;
            }
        }
    }
    else // dist >= 60
    {
        nTurns = 0;
        int directions[] = {1, 1};
        controlledMotors->go(dist/4);
    }
}






void taskerSetup()
{
    tasks[0] = &sonar;
    tasks[1] = &motorPIDs;
    taskDriver = TaskDriver(ntasks, tasks);
    taskDriverp = &taskDriver;
}

void taskerLoop()
{
    taskDriver.run();
}
