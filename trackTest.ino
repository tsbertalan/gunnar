#define TOM_DEBUG // Uncomment to enable debugging serial prints.
#include <Adafruit_MotorShield.h>
#include "pinDefinitions.h"
#include "encoders.h"

Encoder encoder0(encoder0PinA, encoder0PinB);
Encoder encoder1(encoder1PinA, encoder1PinB);

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor* motor1 = AFMS.getMotor(1);
Adafruit_DCMotor* motor2 = AFMS.getMotor(2);

Adafruit_DCMotor* bothMtrs[] = {motor1, motor2};
Adafruit_DCMotor* leftMtrs[] = {motor1};
Adafruit_DCMotor* riteMtrs[] = {motor2};


bool running = false;



const int minimumSensableDistance = 30; // [cm]

unsigned long lastTurn = 0;


 
const int PANOFFSET = 5; // Larger is further to the left.
const int TILTOFFSET = 8; // Larger is further down.
const int NEUTRALPAN = -PANOFFSET;
const int NEUTRALTILT = -TILTOFFSET;

void setup() {
    visionSetup();
    encodersSetup();
    
    pinMode(encoder0PinA, INPUT);
    pinMode(encoder0PinB, INPUT);
    
    pinMode(PIN_ACTIVITYSWITCH, INPUT);
        
    Serial.begin(9600);           // set up Serial library at 9600 bps
    Serial.println("Adafruit Motorshield v2 - DC Motor test!");

    AFMS.begin(1600);

    // Set the speed to start, from 0 (off) to 255 (max speed)

    motor1->run(RELEASE);
    motor2->run(RELEASE);

    setupAccelerometer();

    //   delay(2000);
    //   go(32);
    //   stop();
}

void turn(int angle) {
    int dirs[2];
    if(angle < 0) { // Right turn
        dirs[0] = 1;
        dirs[1] = -1;
    } else {
        dirs[0] = -1;
        dirs[1] = 1;
    }

    int dist = (int) ( (float)(abs(angle)) * 0.38);
    Serial.print("Turn: 'dist'=");
    Serial.print(dist);
    Serial.print(" --> ");
    go(dist, dirs, bothMtrs, 2);
    
    lastTurn = millis();
}


void stop() {
    running = false;
    int directions[] = {0, 0};
    go(0, directions, bothMtrs, 2);
}


const int maxSpeed = 200;  // At 128, we cover 61 cm in 3200 ms
const int rampDelay = 2;  // ms
bool run(int speed, int directions[], Adafruit_DCMotor* mtrs[], int nMtrs) {
    uint8_t i;
    uint8_t m;

    // Set directions, or stop motors.
    int stops = 0;
    for(m=0; m<nMtrs; m++) {
        Adafruit_DCMotor* motor = mtrs[m];
        int dir = directions[m];
        if(dir == 0) {
            motor->run(RELEASE);
            stops++;
        } else {
            if(dir > 0) {
                motor->run(FORWARD);
            } else {
                motor->run(BACKWARD);
            }
        }
    }

    // Ramp up.
    for (i=0; i<speed; i++) {
    for(m=0; m<nMtrs; m++) {
        Adafruit_DCMotor* motor = mtrs[m];
        motor->setSpeed(i);
    }
    interruptibleDelay(rampDelay);
    }
    float a = get2DAccelMag(4);
    Serial.print("Forward running accelaration: ");
    Serial.println(a);
    return a > 20;
}

const float DISTANCEMAGIC = 40.0;  // Make this larger to go farther.
void go(int dist, int directions[], Adafruit_DCMotor* mtrs[], int nMtrs) {
    // dist: how long to run the motor(s).
    // directions: array of length nMtrs of directions to go.
    // mtrs: array of length nMtrs of which motors on which to operate.

    uint8_t i;
    uint8_t m;

    // Set directions, or stop motors.
    int stops = 0;
    for(m=0; m<nMtrs; m++) {
        Adafruit_DCMotor* motor = mtrs[m];
        int dir = directions[m];
        if(dir == 0) {
        motor->run(RELEASE);
        stops++;
        } else {
        if(dir > 0) {
            motor->run(FORWARD);
        } else {
            motor->run(BACKWARD);
        }
        }
    }

    if(stops == nMtrs) {
        // If all motors are stopping, return here.
        return;
    } else {
        // Otherwise, ramp up to speed.
        run(maxSpeed, directions, mtrs, nMtrs);
        
        // Run for a time at this speed.
        float driveTime = DISTANCEMAGIC * dist;
        Serial.print("Go: delaying ");
        Serial.print(driveTime);
        Serial.println(" ms");
        interruptibleDelay(driveTime);
        
        // Ramp back down to zero.
        for (i=maxSpeed; i!=0; i--) {
        for(m=0; m<nMtrs; m++) {
            Adafruit_DCMotor* motor = mtrs[m];
            motor->setSpeed(i);  
        }
        interruptibleDelay(rampDelay);
        }
    }
}

void go(int dist) {
    encoder0.setPosition(0);
    int dir;
    if(dist == 0) {
        dir = 0;
    } else {
        if(dist > 0) {
            dir = 1;
        } else {
            dir = -1;
            dist *= -1;
        }
    }
    int dirs[] = {dir, dir};
    go(dist, dirs, bothMtrs, 2);
    Serial.print("traveled ");
    Serial.print(encoder0.getPosition());
    Serial.println("cm according to optical encoder.");
}

void decideTurn()
{
    decideTurn(-1);
}

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
    
    turn(angle);
}

bool leftTreadBlocked()
{
    return treadBlocked(false);
}
bool rightTreadBlocked()
{
    return treadBlocked(true);
}
bool treadBlocked(bool right)
{
    int blockagePanAngle = 50;
    const int blockageTiltAngle = -60;
    if(right)
    {
        Serial.print("right ");
        blockagePanAngle *= -1;
    }
    else
    {
        Serial.print("left ");
    }
        
    setPan(blockagePanAngle);
    setTilt(blockageTiltAngle);
    interruptibleDelay(4000);
    float dist = getSonarDist();
    Serial.print("ground distance: ");
    Serial.println(dist);
    setPan(0); setTilt(0);
    
    return (dist < 50);
}
    
const int stepDelay = 10;
int nTurns = 0;
void loop() {
    
    if(digitalRead(PIN_ACTIVITYSWITCH) == HIGH)
    {
        setPan(0);
        setTilt(0);
        
        unsigned long now = millis();
        if( running && (now - lastTurn > 16000) )
        {
            // If it's been a while since the last turn, just turn now anyway.
            running = false; // We'll do the decision while moving. Possibly bad.
            decideTurn();
        }
        else
        {
            float dist = getSonarDist(8);
            Serial.print("sighted distance: "); Serial.println(dist);
        //     dist = 4; // prevents forward running.

            if(dist < 60)
            {
                if(dist < minimumSensableDistance)
                {
                    stop();
                    go(-7);
                }
                else // We're not *super* close.githuy
                {
                    if(nTurns > 4)
                    {
                        nTurns = 0;
                        stop();
                        go(-7);
                    }
                    else // We haven't turned very many times.
                    {
                        Serial.println("Obstacle sighted. Turning.");
                        stop();
                        decideTurn();
                        nTurns++;
                    }
                }
            }
            else // dist >= 60
            {
                if(!running)
                {
                    nTurns = 0;
                    int directions[] = {1, 1};
                    run(maxSpeed, directions, bothMtrs, 2);
                    running = true;
                }
            }
        }
    }
    else
    {
        stop();
        disableServos();
    }
    interruptibleDelay(stepDelay);

//     sonarTest();
    
    #ifdef TOM_DEBUG
        Serial.print("v0 = ");
        Serial.println(encoder0.getSpeed());
//         Serial.print("v1 = ");
//         Serial.println(encoder1.getSpeed());
    #endif
}
