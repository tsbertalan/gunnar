#ifndef MOTORS_H
#define MOTORS_H
#include "constants.h"
//motor A connected between A01 and A02
//motor B connected between B01 and B02


void motorStop()
{
  //enable standby  
  digitalWrite(motorPinStby, LOW); 
}


const uint8_t MOTORRELEASE = 0;
const uint8_t MOTORFORWARD = 1;
const uint8_t MOTORBACKWARD = 2;

const boolean MOTORLEFT = LOW;
const boolean MOTORRIGHT = HIGH;

class Motor
{
public:
    void init(boolean which)
    {
        _which = which;
        pinMode(motorPinStby, OUTPUT);
        if(_which == 0)
        {
            pinMode(motorPinPwmA, OUTPUT);
            pinMode(motorPinAin1, OUTPUT);
            pinMode(motorPinAin2, OUTPUT);
        }
        else
        {
            pinMode(motorPinPwmB, OUTPUT);
            pinMode(motorPinBin1, OUTPUT);
            pinMode(motorPinBin2, OUTPUT);
        }
        _status = MOTORRELEASE;
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
            
        _status = status;
        
        if(status == MOTORRELEASE)
        {
            digitalWrite(motorPinStby, LOW); //enable standby
            return;
        }
        else
        {
            digitalWrite(motorPinStby, HIGH); //disable standby
        }
        
        boolean inPin1 = HIGH;
        boolean inPin2 = LOW;
        
        if(_status == MOTORBACKWARD)
        {
//             Serial.print("going backward : motor ");
//             Serial.println(_which);
            inPin1 = LOW;
            inPin2 = HIGH;
        }
        
        for(uint8_t i=0; i<50; i++)
        {
            noInterrupts();
            if(_which == MOTORLEFT)
            {
                digitalWrite(motorPinAin1, inPin1);
                digitalWrite(motorPinAin2, inPin2);
            }
            else
            {
                digitalWrite(motorPinBin1, inPin1);
                digitalWrite(motorPinBin2, inPin2);
            }
            interrupts();
        }
    }
  
};


#endif