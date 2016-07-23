#ifndef MOTORS_H
#define MOTORS_H
#include "constants.h"
//motor A connected between A01 and A02
//motor B connected between B01 and B02


const uint8_t MOTORSTOP = 0;
const uint8_t MOTORFORWARD = 1;
const uint8_t MOTORBACKWARD = 2;

// TODO: Use ENUMs for all these identifier constants.
const boolean MOTORLEFT = LOW;
const boolean MOTORRIGHT = HIGH;

class Motor {
public:
    void init(boolean which) {
        if(which == MOTORLEFT) {
            _dirPin = motorPinDirA;
            _pwmPin = motorPinPwmA;
            _curPin = motorPinCurA;
        } else {
            _dirPin = motorPinDirB;
            _pwmPin = motorPinPwmB;
            _curPin = motorPinCurB;
        }
        pinMode(_pwmPin, OUTPUT);
        pinMode(_dirPin, OUTPUT);
        pinMode(_curPin, INPUT);
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

        analogWrite(_pwmPin, speed);
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
    uint16_t _speed;
    uint8_t _status;

    uint8_t _dirPin;
    uint8_t _pwmPin;
    uint8_t _curPin;

    void _setStatus(uint8_t status) {

        if(status == MOTORSTOP) {
            stop();
        } else {
            digitalWrite(_dirPin, status==MOTORBACKWARD);
        }

        _status = status;
    }

};


#endif
