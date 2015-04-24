#include "constants.h"
//motor A connected between A01 and A02
//motor B connected between B01 and B02

void setup(){
  Serial.begin(9600);
  pinMode(motorPinStby, OUTPUT);

  pinMode(motorPinPwm,A OUTPUT);
  pinMode(motorPinAin1, OUTPUT);
  pinMode(motorPinAin2, OUTPUT);

  pinMode(motorPinPwmB, OUTPUT);
  pinMode(motorPinBin1, OUTPUT);
  pinMode(motorPinBin2, OUTPUT);
}

void loop(){
  uint8_t i;
  for(int8_t dir=1; dir>=0; dir--)
  {
    Serial.print("direction is ");
    Serial.println(dir);
    for(i=1; i<255; i++)
    {
        move(1, i, dir);
        move(2, i, dir);
        delay(10);
        Serial.print("speed is ");
        Serial.println(i);
    }
    for(i=255; i>0; i--)
    {
        move(1, i, dir);
        move(2, i, dir);
        delay(10);
        Serial.print("speed is ");
        Serial.println(i);
    }
  }
}


void move(int motor, int speed, int direction){
  //Move specific motor at speed and direction
  //motor: 0 for B 1 for A
  //speed: 0 is off, and 255 is full speed
  //direction: 0 clockwise, 1 counter-clockwise

  digitalWrite(motorPinStby, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if(motor == 1){
    digitalWrite(motorPinAin1, inPin1);
    digitalWrite(motorPinAin2, inPin2);
    analogWrite(motorPinPwmA , speed);
  }
  else{
    digitalWrite(motorPinBin1, inPin1);
    digitalWrite(motorPinBin2, inPin2);
    analogWrite(motorPinPwmB, speed);
  }
}

void stop(){
  //enable standby  
  digitalWrite(motorPinStby, LOW); 
}


