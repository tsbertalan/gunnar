#include <Servo.h>
#include <gunnar.h>
const int daguPwmPin1 = 11;
const int daguPwmPin2 = 12;

const int daguDirPin1 = 44;
const int daguDirPin2 = 45;

const int daguCurPin1 = A0;
const int daguCurPin2 = A1;

void setup()
{
    Serial.begin(9600);
    Serial.println("dagu motor board test");
    pinMode(daguPwmPin1, OUTPUT);
    pinMode(daguPwmPin2, OUTPUT);
    pinMode(daguDirPin1, OUTPUT);
    pinMode(daguDirPin2, OUTPUT);
    pinMode(daguCurPin1, INPUT);
    pinMode(daguCurPin2, INPUT);
}

void rampUpDown(boolean direction)
{
    digitalWrite(daguDirPin1, direction);
    digitalWrite(daguDirPin2, direction);
    
    uint8_t speed;

    for(speed=0; speed<255; speed++)
    {
        Serial.print("increasing: ");
        Serial.print(speed);
        Serial.print(" ");
        Serial.print(analogRead(daguCurPin1));
        Serial.print(" ");
        Serial.println(analogRead(daguCurPin2));
        analogWrite(daguPwmPin1, speed);
        analogWrite(daguPwmPin2, speed);
        delay(4);
    }
    
    for(speed=254; speed>=1; speed--)
    {
        Serial.print("decreasing: ");
        Serial.print(speed);
        Serial.print(" ");
        Serial.print(analogRead(daguCurPin1));
        Serial.print(" ");
        Serial.println(analogRead(daguCurPin2));
        analogWrite(daguPwmPin1, speed);
        analogWrite(daguPwmPin2, speed);
        delay(4);
    }  
}

void loop()
{
    rampUpDown(HIGH);
    rampUpDown(LOW);
}