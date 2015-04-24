#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor* motors[] = {
  AFMS.getMotor(1),
  AFMS.getMotor(2),
  AFMS.getMotor(3),
  AFMS.getMotor(4)
};


void setup() {
  AFMS.begin(1600);
  for(int i=0; i<4; i++) {
    motors[i]->run(FORWARD);
    motors[i]->setSpeed(64);
  }
}

void loop() {
  delay(1000);       
}
