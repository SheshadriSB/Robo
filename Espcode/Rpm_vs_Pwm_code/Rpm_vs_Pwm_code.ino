#define PWM_PIN 13
#define DIR_PIN1 12
#define DIR_PIN2 14
#define ENCODER_PIN 32
#define PULSES_PER_REV 210

volatile int pulseCount = 0;

void IRAM_ATTR countPulse() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countPulse, RISING);

  pinMode(DIR_PIN1, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);

  digitalWrite(DIR_PIN1, HIGH);
  digitalWrite(DIR_PIN2, LOW);

  Serial.println("Starting data collection");
}

void loop() {
  for (int freq =8000 ; freq <=12000; freq += 500) {
    for (int run = 0; run < 1; run++) { // Repeat 3 times for each frequency
      ledcDetach(PWM_PIN);
      ledcAttach(PWM_PIN, freq, 8); // Setup PWM for given frequency
      ledcWrite(PWM_PIN, 0); // Stop the motor initially
      delay(2000); // Pause before measurement

      for (int dutyValue = 0; dutyValue <= 255; dutyValue += 1) {
        ledcWrite(PWM_PIN, dutyValue);

        unsigned long startTime = millis();
        pulseCount = 0;

        while (millis() - startTime < 100) {
          // Wait 100 ms for stable RPM measurement
        }

        noInterrupts();
        float motorRPM = (pulseCount * 600.0) / PULSES_PER_REV; // RPM calculation
        interrupts();

        Serial.print(freq);
        Serial.print(",");
        Serial.print(dutyValue);
        Serial.print(",");
        Serial.print(run + 1); // Run number
        Serial.print(",");
        Serial.println(motorRPM);
      }



        for (int dutyValue = 255; dutyValue >= 0; dutyValue -= 1) {
        ledcWrite(PWM_PIN, dutyValue);

        unsigned long startTime = millis();
        pulseCount = 0;

        while (millis() - startTime < 100) {
          // Wait 100 ms for stable RPM measurement
        }

        noInterrupts();
        float motorRPM = (pulseCount * 600.0) / PULSES_PER_REV; // RPM calculation
        interrupts();

        Serial.print(freq);
        Serial.print(",");
        Serial.print(dutyValue);
        Serial.print(",");
        Serial.print(run + 1); // Run number
        Serial.print(",");
        Serial.println(motorRPM);
      }




    }
  }

  Serial.println("Data collection finished");
  delay(5000); // Delay before restarting
  ESP.restart(); // Restart ESP32
}
