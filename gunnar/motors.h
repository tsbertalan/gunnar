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

class Motor {
public:
    void init(boolean which) {
        pinMode(motorPinPwmA, OUTPUT);
        pinMode(motorPinDirA, OUTPUT);
        pinMode(motorPinCurA, INPUT);
        pinMode(motorPinPwmB, OUTPUT);
        pinMode(motorPinDirB, OUTPUT);
        pinMode(motorPinCurB, INPUT);
        _which = which;
        stop();
    }

    void stop() {
        setSpeed(0);
        _status = MOTORSTOP;
    }

    void run(uint8_t status) {
        _setStatus(status);
    }

    void setSpeed(int speed) {
        if(speed < 0) {
            _setStatus(MOTORBACKWARD);
            speed = abs(speed);
        } else {
            _setStatus(MOTORFORWARD);
        }
        _speed = constrain(speed, 0, 255);

//         Serial.print("Setting motor ");
//         Serial.print(_which);
//         Serial.print(" to ");
//         Serial.print(speed);
//         Serial.println(".");
        if(_which == MOTORLEFT) {
            analogWrite(motorPinPwmA, speed);
        } else {
            analogWrite(motorPinPwmB, speed);
        }
    }

    uint16_t getSpeed() {
        return _speed;
    }

    int16_t getSpeedSigned() {
        if(getStatus() == MOTORBACKWARD)
            return -1 * (int16_t) getSpeed();
        else
            return getSpeed();
    }

    uint8_t getStatus() {
        return _status;
    }

private:
    boolean _which;
    uint16_t _speed;
    uint8_t _status;

    void _setStatus(uint8_t status) {

        if(status == MOTORSTOP) {
            stop();
        } else {
            // Since both motors are wired the same, but face different
            // sides of the vehicle, the sense of "forward" is different
            // for both.
            if(_which == MOTORLEFT) {
                digitalWrite(motorPinDirA, status==MOTORFORWARD);
            } else {
                digitalWrite(motorPinDirB, status==MOTORBACKWARD);
            }
        }

        _status = status;
    }

};


#endif
