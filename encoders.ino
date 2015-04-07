#include "encoders.h"
#include "stats.h"
void encodersSetup()
{
    attachInterrupt(0, doEncoder0, RISING);
    attachInterrupt(1, doEncoder1, RISING);
    pinMode(encoder0PinA, INPUT);
    pinMode(encoder0PinB, INPUT);
} 

float _leftMotorSpeed;
float _rightMotorSpeed;

void interruptibleDelay(float ms) {
    // Replace delay(ms) with multiple delayMicroseconds(...), so interrupts still work.
    const int maxDelay = 16383; // delayMicroseconds is only accurate up to this long (and down to ~3 uS).
    
    int nreps = floor(ms*1000 / maxDelay);
    
    // First, delay the remainder. For small ms, the function could stop here.
    delayMicroseconds(fmod(ms*1000, maxDelay));

    // Delay the necessary remaining repetitions of maxDelay.
    for(uint8_t i=0; i<nreps; i++)
    {
        delayMicroseconds(maxDelay);
    }    
}

void doEncoder0(){
    encoder0.update();
}

void doEncoder1(){
    encoder1.update();
}
