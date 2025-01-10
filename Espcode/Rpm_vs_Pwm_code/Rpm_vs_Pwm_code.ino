#define Motor1_Pin 2   // Encoder Pin for Motor 1 (must be interrupt-capable)
#define PWM_Pin 5     // PWM Pin for Motor 1 (Arduino supports PWM on specific pins)

volatile int pulseCount = 0;   // Pulse count for motor speed
float MotorRpm = 0;            // Motor speed in RPM
const int PPR = 210;           // Pulses per revolution (encoder specification)

int pwmSignal = 0;             // PWM signal value (0-255)
unsigned long lastTime = 0;    // Last time for RPM calculation
unsigned long calcInterval = 100; // Interval for RPM calculation (milliseconds)

void setup() {
  Serial.begin(115200);

  // Motor Encoder Pin setup
  pinMode(Motor1_Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Motor1_Pin), updatePulse, RISING);

  // PWM Pin setup
  pinMode(PWM_Pin, OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  digitalWrite(6,HIGH);
  digitalWrite(7,LOW);

  // Initialize PWM signal (set to 0)
  analogWrite(PWM_Pin, pwmSignal);
}

void loop() {
  // Calculate RPM
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= calcInterval) {
    MotorRpm = (pulseCount * 600.0) / PPR; // RPM = pulses per minute / pulses per revolution
    pulseCount = 0;                       // Reset pulse count
    lastTime = currentTime;

    // Print data for plotting
    Serial.print(pwmSignal); // X-axis: PWM
    Serial.print(",");
    Serial.println(MotorRpm); // Y-axis: RPM
  }

  // Update PWM signal dynamically (example: increasing and decreasing)
  pwmSignal += 1.5; 
  if (pwmSignal > 255) pwmSignal = 0; // Reset PWM when it exceeds 255
  analogWrite(PWM_Pin, pwmSignal);

  delay(50); // Small delay for smoother transitions
}

void updatePulse() {
  pulseCount++; // Increment pulse count on each encoder tick
}
