

double freqs[] = {1, 2, 10, 100, 1000, 5000, 10000, 20000};
// double freqs[] = {1, 2, 3,  4,   5,    6,    7, 8};
// int pin = 13;

int pin = 12;
void setup() {
  pinMode(pin, OUTPUT);
  Serial.begin(9600);
}

void delayUsOrMs(long us) {
  if(us > 16383) {
    long ms = (long) ((double) us / 1000.0);
    delay(ms);
  } else {
    delayMicroseconds(us);
  }
}


void loop() {
  
  // Bit-bang the frequencies so we don't break delay().
  for(int ifreq=0; ifreq<8; ifreq++) {
    double freq = freqs[ifreq];
    
    long periodUs = 1.0 / freq * 1000 * 1000;
    Serial.print("Period is ");
    Serial.print(periodUs);
    Serial.println(" microseconds.");
    int nsecs = 2;
    long ncycs = nsecs * freq;
    bool on = 0;
    
    for(int cyc=0; cyc<ncycs; cyc++) {
      if( on ) {
        on = 0;
      } else {
        on = 1;
      }
      digitalWrite(pin, on);
      delayUsOrMs(periodUs);
    }
  }
}
