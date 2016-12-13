#ifndef DaguRover5MotorDriver_h
#define DaguRover5MotorDriver_h

#include <Arduino.h>

class DaguRover5MotorDriver
{
  public:
    // CONSTRUCTORS
	DaguRover5MotorDriver(); // Default pin selection.
	DaguRover5MotorDriver(unsigned char M1DIR, unsigned char M1PWM, unsigned char M1FB,
                          unsigned char M2DIR, unsigned char M2PWM, unsigned char M2FB); // User-defined pin selection.

    // PUBLIC METHODS
    void init(); // Initialize TIMER 1, set the PWM to 20kHZ.
    void setM1Speed(int speed); // Set speed for M1.
    void setM2Speed(int speed); // Set speed for M2.
    void setSpeeds(int m1Speed, int m2Speed); // Set speed for both M1 and M2.
    unsigned int getM1CurrentMilliamps(); // Get current reading for M1.
    unsigned int getM2CurrentMilliamps(); // Get current reading for M2.

  private:
    unsigned char _M1DIR;
    unsigned char _M2DIR;
    static const unsigned char _M1PWM = 9;
    static const unsigned char _M2PWM = 10;
    unsigned char _M1FB;
    unsigned char _M2FB;
};

#endif
