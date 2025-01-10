#define Motor1_Pin 2   
#define PWM_Pin 5     

volatile int pulseCount = 0;   
float MotorRpm = 0;            
const int PPR = 210;           

int pwmSignal = 0;            
unsigned long lastTime = 0;   
unsigned long calcInterval = 100; 

void setup() {
  Serial.begin(115200);


  pinMode(Motor1_Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Motor1_Pin), updatePulse, RISING);

 
  pinMode(PWM_Pin, OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  digitalWrite(6,HIGH);
  digitalWrite(7,LOW);

  
  analogWrite(PWM_Pin, pwmSignal);
}

void loop() {
  
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= calcInterval) {
    MotorRpm = (pulseCount * 600.0) / PPR; 
    pulseCount = 0;                      
    lastTime = currentTime;

   
    Serial.print(pwmSignal); 
    Serial.print(",");
    Serial.println(MotorRpm); 
  }

  
  pwmSignal += 1.5; 
  if (pwmSignal > 255) pwmSignal = 0; 
  analogWrite(PWM_Pin, pwmSignal);

  delay(50); 
}

void updatePulse() {
  pulseCount++; 
}
