#ifndef UITLS_H
#define UTILS_H
#include <math.h>
#include <Arduino.h>

void interruptibleDelay(float ms)
{
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

float average(float* values, int N)
{
    float val = 0;
    for(int i=0; i<N; i++)
    {
        val += values[i];
    }
    val /= (float) N;
    return val;
}

#endif