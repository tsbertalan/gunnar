#ifndef ENCODER_H
#define ENCODER_H

int encoder0Pos; // Standard should ensure this is initialized to 0.
int encoder0PinALast = LOW;
int n = LOW;
const int nspeeds = 4; // number of previous speeds to average
const long SPEEDSCALE = 256000L; // We need to keep the numerator and denominator similarly scaled.
const int NONINTERRUPTDELAY = 30000; // [us]

class Encoder
{
  /*  
    wraps encoder setup and update functions in a class

    !!! NOTE : user must call the encoders update method from an
    interrupt function himself!
  */
public:

    // constructor : sets pins as inputs and turns on pullup resistors
    Encoder( int8_t PinA, int8_t PinB) : pin_a ( PinA), pin_b( PinB )
    {
        // set pin a and b to be input 
        pinMode(pin_a, INPUT); 
        pinMode(pin_b, INPUT); 
        // and turn on pullup resistors
        digitalWrite(pin_a, HIGH);    
        digitalWrite(pin_b, HIGH);
        
         // These might not be necessary:
        lastPosition = 0;
        lastPositionMicros = 0;
    };

    
    // Call this from other code, or with a timer/internal interrupt.
    void nonInterruptUpdate()
    {
//         setSpeed(SPEEDSCALE / (micros() - lastPositionMicros));
        if(micros() - lastPositionMicros > NONINTERRUPTDELAY)
        {
            // Conclude that nothing's moving.
            setSpeed(0);
        }
    }
    
    // Call this from your interrupt function.
    void update()
    {
        if(
            (digitalRead(pin_a) && digitalRead(pin_b))
            ||
            (!digitalRead(pin_a) && !digitalRead(pin_b))
            )
            position--;
        else
            position++;
        
        unsigned long long nowPositionMicros = micros(); // Use micros to not mess up the interrupts.
        float speed = (float)(position - lastPosition) * SPEEDSCALE  / (nowPositionMicros - lastPositionMicros);
        if(abs(speed) < 50) // outlier rejection
            setSpeed(speed);
        lastPosition = position;
        lastPositionMicros = nowPositionMicros;
    };

    void setSpeed(float speed)
    {
        speedNow = speed;
        dspeedNow = (double) speedNow;
        for(uint8_t i=0; i<nspeeds-1; i++)
        {
            speedHist[i] = speedHist[i+1];
        }
        speedHist[nspeeds-1] = speedNow;
        
        speedNowAvg = average(speedHist, nspeeds);
        dspeedNowAvg = (double) speedNowAvg;
    }
    
    long getPosition()
    {
        return (long) position;
    };

    
    void setPosition(const long int p)
    {
        position = p;
    };
    
    float getSpeed()
    {
        return speedNowAvg;
    }
    
//     double getSpeed()
//     {
//         return dspeedNowAvg;
//     }
    
    
    float speedNow;
    double dspeedNow;
    
    float speedNowAvg;
    double dspeedNowAvg;

private:
    float speedHist[nspeeds];
    long long position;
    int8_t pin_a;
    int8_t pin_b;
    long long lastPosition;
    unsigned long long lastPositionMicros; // micros() overflows after 70 minutes.
    // I don't think our batteries will last that long, and certainly
    // not our quiescences.
};

#endif
