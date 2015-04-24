#include <Servo.h> 
#include <MemoryFree.h>
#include <gunnar.h>

Servo tiltServo; 
Servo panServo;

int pos = 0;

void setup() 
{ 
  Serial.begin(BAUDRATE);
  Serial.println("SERVO TEST");
  tiltServo.attach(TILTSERVOPIN);
  panServo.attach(PANSERVOPIN);
} 


const int MAXPOS = 120;
const int MINPOS = 40;
void loop() 
{ 
  Serial.println("          [          ]");
  Serial.print("Tilt test: ");
  for(pos = MINPOS; pos < MAXPOS; pos += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    tiltServo.write(pos);
    delay(15);                       // waits 15ms for the servo to reach the position 
    if(pos%((MAXPOS-MINPOS)/5) == 0)
      Serial.print("|");
  } 
  for(pos = MAXPOS; pos>=MINPOS+1; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    tiltServo.write(pos);
    delay(15);                       // waits 15ms for the servo to reach the position 
    if(pos%((MAXPOS-MINPOS)/5) == 0)        Serial.print("|");
  }
  Serial.println(" done.");

  Serial.println("          [         ]");
  Serial.print("Pan test:  ");
  for(pos = MINPOS; pos < MAXPOS; pos += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    panServo.write(pos);
    delay(25);                       // waits 15ms for the servo to reach the position 
    if(pos%((MAXPOS-MINPOS)/5) == 0)
      Serial.print("|");
  } 
  for(pos = MAXPOS; pos>=MINPOS+11; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    panServo.write(pos);
    delay(25);                       // waits 15ms for the servo to reach the position 
    if(pos%((MAXPOS-MINPOS)/5) == 0)
      Serial.print("|");
  } 
  Serial.println(" done.");
} 

