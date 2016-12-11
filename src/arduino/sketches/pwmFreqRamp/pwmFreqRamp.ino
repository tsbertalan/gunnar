

double freqs[] = {31, 100, 1000, 5000, 10000, 20000, 40000, 60000};
int pin = 12;

void setup() {
  pinMode(pin, OUTPUT);
  Serial.begin(9600);
}

void loop() {  
  for(int ifreq=0; ifreq<8; ifreq++) {
    double freq = freqs[ifreq];
    
    Serial.print("Frequency is ");
    Serial.print(freq);
    Serial.println("Hz.");
    
    int nsecs = 8;
    tone(pin, freq, nsecs * 1000);
    delay(nsecs * 1000);
  }
}
