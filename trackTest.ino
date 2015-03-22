/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/

#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor* motor1 = AFMS.getMotor(1);
Adafruit_DCMotor* motor2 = AFMS.getMotor(2);

Adafruit_DCMotor* bothMtrs[] = {motor1, motor2};
Adafruit_DCMotor* leftMtrs[] = {motor1};
Adafruit_DCMotor* riteMtrs[] = {motor2};

const int encoder0PinA = 6;
const int encoder0PinB = 7;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
int nEnc = LOW;

bool running = false;

const int PIN_ACTIVITYSWITCH = 3;

const int minimumSensableDistance = 30; // [cm]

unsigned long lastTurn = 0;

void setup() {
    visionSetup();
    
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


void updateEncoderPositions() {
    nEnc = digitalRead(encoder0PinA);
    if ((encoder0PinALast == LOW) && (nEnc == HIGH)) {
        if(digitalRead(encoder0PinB) == LOW) {
        encoder0Pos--;
        } else {
        encoder0Pos++;
        }
    } 
    encoder0PinALast = nEnc;
}

const float encRate = 32.0 / 129.0;  // cm per encoder tick
float getPosition() {
    return encRate * encoder0Pos;
}

void setPosition(float x) {
    encoder0Pos = x / encRate;
}

void encodedDelay(float ms) {
    delay(ms);
//     float encRepeatRate = 10.0;
//     if(ms >= encRepeatRate) {
//         Serial.print("delaying ");
//         Serial.print(ms);
//         Serial.print(" ms ... ");
//     }
//     float rem = fmod(ms, encRepeatRate);
//     int reps = (ms - rem) / encRepeatRate;
//     uint8_t i;
//     for(i=0; i<reps; i++) {
//         delay(encRepeatRate);
//         updateEncoderPositions();
//     }
//     if(ms > encRepeatRate)
//         Serial.println("done");
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
        updateEncoderPositions();
    }
    encodedDelay(rampDelay);
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
        encodedDelay(driveTime);
        
        // Ramp back down to zero.
        for (i=maxSpeed; i!=0; i--) {
        for(m=0; m<nMtrs; m++) {
            Adafruit_DCMotor* motor = mtrs[m];
            motor->setSpeed(i);  
        }
        encodedDelay(rampDelay);
        }
    }
}

void go(int dist) {
    setPosition(0);
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
    Serial.print(getPosition());
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
    setTilt(-20); // Look down slightly.
    
    setPan(-45);
    encodedDelay(delayBeforeMeasurement);
    float rdist = getSonarDist(nMeasurements);
//     Serial.print("rdist: ");
//     Serial.println(rdist);
    
    setPan(45);
    encodedDelay(delayBeforeMeasurement);
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
    
//     encodedDelay(500);
    
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
    encodedDelay(4000);
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
        unsigned long now = millis();
        if( running && (now - lastTurn > 5000) )
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
                else // We're not *super* close.
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
    }
    encodedDelay(stepDelay);

//     sonarTest();
}
