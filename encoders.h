#ifndef ENCODER_H
#define ENCODER_H

int encoder0Pos; // Standard should ensure this is initialized to 0.
int encoder0PinALast = LOW;
int n = LOW;
const int nspeeds = 5; // number of previous speeds to average

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
        lastPositionMillis = 0;
        for(uint8_t i=0; i<nspeeds; i++)
        {
            speeds[i] = 0;
        }
    };

    
    // call this from your interrupt function
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
        
        float nowPositionMillis = micros() / 1000.0; // Use micros to not mess up the interrupts.
        float speedNow = (float)(position - lastPosition) * 1000.0  / (nowPositionMillis - lastPositionMillis);
        updateSpeeds(speedNow);
               
        lastPosition = position;
        lastPositionMillis = nowPositionMillis;
    };

    
    long int getPosition()
    {
        return position;
    };

    
    void setPosition(const long int p)
    {
        position = p;
    };
    
    float getSpeed()
    {
        float out = 0.0;
        for(uint8_t i=0; i<nspeeds; i++)
        {
            out += speeds[i];
        }
        return out / (float) nspeeds;
    }
    
    void updateSpeeds(float speedNow)
    {
        for(uint8_t i=0; i<nspeeds-1; i++)
        {
            speeds[i] = speeds[i+1];
        }
        speeds[nspeeds-1] = speedNow;
    }
            

private:
    long int position;
    int8_t pin_a;
    int8_t pin_b;
    int lastPosition;
    unsigned long lastPositionMillis; // micros() overflows after 70 minutes.
    // I don't think our quiescences will last that long, and certainly
    // not our quiescences.
    int speeds[nspeeds];
};

#endif