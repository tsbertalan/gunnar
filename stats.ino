#include "stats.h"
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

float averageWithoutkOutliers(float* valuse, int N, int k)
{
}