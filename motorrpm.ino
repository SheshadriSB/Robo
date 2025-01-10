volatile long pulseCount = 0;  
unsigned long lastTime = 0;     
const int ppr = 210;              
const int encoderPin = 2;        


void countPulses() {
  pulseCount++; 
}

void setup() {
  pinMode(encoderPin, INPUT_PULLUP);                  
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulses, RISING); 
  Serial.begin(9600);                                 
}

void loop() {
  unsigned long currentTime = millis();              

  if (currentTime - lastTime >= 1000) {              
    noInterrupts();                                
    long pulses = pulseCount;                     
    pulseCount = 0;                                 
    interrupts();                                  

    float rpm = (pulses * 60.0) / ppr;              
    Serial.print("RPM: ");
    Serial.println(rpm);                            

    lastTime = currentTime;                        
  }
}
