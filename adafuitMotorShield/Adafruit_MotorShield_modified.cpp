/******************************************************************
 This is the library for the Adafruit Motor Shield V2 for Arduino. 
 It supports DC motors & Stepper motors with microstepping as well
 as stacking-support. It is *not* compatible with the V1 library!

 It will only work with https://www.adafruit.com/products/1483
 
 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source hardware
 by purchasing products from Adafruit!
 
 Written by Limor Fried/Ladyada for Adafruit Industries.
 BSD license, check license.txt for more information.
 All text above must be included in any redistribution.
 ******************************************************************/


#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include <Wire.h>
#include "Adafruit_MotorShield_modified.h"
#include "Adafruit_PWMServoDriver.h"
#ifdef __AVR__
 #define WIRE Wire
#else // Arduino Due
 #define WIRE Wire1
#endif


#if (MICROSTEPS == 8)
uint8_t microstepcurve[] = {0, 50, 98, 142, 180, 212, 236, 250, 255};
#elif (MICROSTEPS == 16)
uint8_t microstepcurve[] = {0, 25, 50, 74, 98, 120, 141, 162, 180, 197, 212, 225, 236, 244, 250, 253, 255};
#endif

Adafruit_MotorShield::Adafruit_MotorShield(uint8_t addr) {
  _addr = addr;
  _pwm = Adafruit_PWMServoDriver(_addr);
}

void Adafruit_MotorShield::begin(uint16_t freq) {
  // init PWM w/_freq
  WIRE.begin();
  _pwm.begin();
  _freq = freq;
  _pwm.setPWMFreq(_freq);  // This is the maximum PWM frequency
  for (uint8_t i=0; i<16; i++) 
    _pwm.setPWM(i, 0, 0);
}

void Adafruit_MotorShield::setPWM(uint8_t pin, uint16_t value) {
  if (value > 4095) {
    _pwm.setPWM(pin, 4096, 0);
  } else 
    _pwm.setPWM(pin, 0, value);
}
void Adafruit_MotorShield::setPin(uint8_t pin, boolean value) {
  if (value == LOW)
    _pwm.setPWM(pin, 0, 0);
  else
    _pwm.setPWM(pin, 4096, 0);
}

Adafruit_DCMotor *Adafruit_MotorShield::getMotor(uint8_t num) {
  if (num > 4) return NULL;

  num--;

  if (dcmotors[num].motornum == 0) {
    // not init'd yet!
    dcmotors[num].motornum = num;
    dcmotors[num].MC = this;
    uint8_t pwm, in1, in2;
    // TODO: Change these pins. We need 10-13 and 3-5 for wifi.
    // 5 maybe for startup only. Wifi could also use 2 in place of 3.
    // If I only need two motors, maybe I just need two triples of pwm/in2/in1:
    // 2, 7, or 8 / 3, 6, or 9 / all bad. Maybe try some of the other unused ones, or one of the 3,6,9?
    if (num == 0) {
      pwm = 8; in2 = 9; in1 = 10;
                                    } else if (num == 1) {
      pwm = 13; in2 = 12; in1 = 11;
                                    } else if (num == 2) {
      pwm = 2; in2 = 3; in1 = 4;
                                    } else if (num == 3) {
      pwm = 7; in2 = 6; in1 = 5;
    }
    dcmotors[num].PWMpin = pwm;
    dcmotors[num].IN1pin = in1;
    dcmotors[num].IN2pin = in2;
  }
  return &dcmotors[num];
}

/******************************************
               MOTORS
******************************************/

Adafruit_DCMotor::Adafruit_DCMotor(void) {
  MC = NULL;
  motornum = 0;
  PWMpin = IN1pin = IN2pin = 0;
}

uint8_t Adafruit_DCMotor::getStatus() {
  return status;
}

void Adafruit_DCMotor::run(uint8_t cmd) {
  status = cmd;
  switch (cmd) {
  case FORWARD:
    MC->setPin(IN2pin, LOW);  // take low first to avoid 'break'
    MC->setPin(IN1pin, HIGH);
    break;
  case BACKWARD:
    MC->setPin(IN1pin, LOW);  // take low first to avoid 'break'
    MC->setPin(IN2pin, HIGH);
    break;
  case RELEASE:
    MC->setPin(IN1pin, LOW);
    MC->setPin(IN2pin, LOW);
    break;
  }
}

void Adafruit_DCMotor::setSpeed(uint8_t speed) {
  MC->setPWM(PWMpin, speed*16);
}
