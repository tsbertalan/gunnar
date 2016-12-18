/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A PD2  //pin (uno 2) (mega 19)
  #define LEFT_ENC_PIN_B PD3  //pin (uno 3) (mega 18)
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A PC4  //pin (uno A4) (mega A12)
  #define RIGHT_ENC_PIN_B PC5  //pin (uno A5) (mega A13)
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

