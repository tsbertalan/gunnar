/* This file was automatically generated.  Do not edit! */
void loop();
void toggleLed();
bool hasExpired(unsigned long&prevTime,unsigned long interval);
void setup();
void attachCommandCallbacks();
void OnFloatAddition();
void OnArduinoReady();
void OnUnknownCommand();
extern CmdMessenger cmdMessenger;
extern const int kBlinkLed;
extern bool ledState;
extern unsigned long previousToggleLed;
