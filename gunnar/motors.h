#ifndef MOTORS_H
#define MOTORS_H
#include "constants.h"
//motor A connected between A01 and A02
//motor B connected between B01 and B02


const uint8_t MOTORSTOP = 0;
const uint8_t MOTORFORWARD = 1;
const uint8_t MOTORBACKWARD = 2;

const boolean MOTORLEFT = LOW;
const boolean MOTORRIGHT = HIGH;

class Motor
{
public:
    void init(boolean which)
    {
        pinMode(motorPinPwmA, OUTPUT);
        pinMode(motorPinDirA, OUTPUT);
        pinMode(motorPinCurA, INPUT);
        pinMode(motorPinPwmB, OUTPUT);
        pinMode(motorPinDirB, OUTPUT);
        pinMode(motorPinCurB, INPUT);
        _which = which;
        stop();
    }
    
    void stop()
    {
        setSpeed(0);
        _status = MOTORSTOP;
    }
    
    void run(uint8_t status)
    {
        _setStatus(status);
    }
    
    void setSpeed(int speed)
    {
        if(speed < 0)
        {
            _setStatus(MOTORBACKWARD);
            speed = abs(speed);
        }
        else
        {
            _setStatus(MOTORFORWARD);
        }
        _speed = speed;
        
        if(_which == MOTORLEFT)
        {
            analogWrite(motorPinPwmA, speed);
        }
        else
        {
            analogWrite(motorPinPwmB, speed);
        }
    }
    
    uint8_t getSpeed()
    {
        return _speed;
    }
    
    uint8_t getStatus()
    {
        return _status;
    }
        
private:
    boolean _which;
    uint8_t _speed;
    uint8_t _status;
    
    void _setStatus(uint8_t status)
    {
        if(status != _status)
        {
            Serial.print(micros());
            Serial.print(" changing direction of m");
            Serial.print(_which);
            Serial.print(" from ");
            Serial.print(_status);
            Serial.print(" to ");
            Serial.println(status);
        }
            
        if(status == MOTORSTOP)
        {
            stop();
        }
        else
        {
            // Since both motors are wired the same, but face different
            // sides of the vehicle, the sense of "forward" is different
            // for both.
            if(_which == MOTORLEFT)
            {
                digitalWrite(motorPinDirA, status==MOTORBACKWARD);
            }
            else
            {
                digitalWrite(motorPinDirB, status==MOTORFORWARD);
            }
        }
        
        _status = status;
    }
  
};


#endif