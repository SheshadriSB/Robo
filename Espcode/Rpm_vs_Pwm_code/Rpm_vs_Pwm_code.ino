#define ENCODER_A 35 // Pin for encoder channel A (must be interrupt-capable)
#define ENCODER_B 32 // Pin for encoder channel B (must be digital input)
#define PULSES_PER_REVOLUTION 600 // Adjust based on your encoder specification

volatile long pulseCount = 0; // Total pulse count
volatile int direction = 1;  // 1 for CW, -1 for CCW
volatile unsigned long lastTimeMicros = 0; // Time of the last pulse in microseconds
volatile float rpm = 0.0; // Calculated RPM

void setup() {
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  // Attach interrupts for channel A
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
  
  Serial.begin(9600);
}

void loop() {
  // Display the current RPM and direction
  noInterrupts(); // Temporarily disable interrupts to access shared variables
  float currentRPM = rpm;
  int currentDirection = direction;
  interrupts(); // Re-enable interrupts

  Serial.print("RPM: ");
  Serial.print(currentRPM);
  Serial.print(" Direction: ");
  if (currentDirection == 1) {
    Serial.println("CW");
  } else {
    Serial.println("CCW");
  }

  delay(5); // Update every 500ms (optional)
}

// Interrupt Service Routine for encoder channel A
void encoderISR() {
  unsigned long currentTimeMicros = micros(); // Get current time in microseconds
  int stateA = digitalRead(ENCODER_A);
  int stateB = digitalRead(ENCODER_B);

  // Determine direction based on A and B signals
  if (stateA == stateB) {
    direction = -1; // Counterclockwise
  } else {
    direction = 1; // Clockwise
  }

  // Calculate RPM based on time between pulses
  if (lastTimeMicros > 0) {
    unsigned long timeDeltaMicros = currentTimeMicros - lastTimeMicros;
    rpm = (60.0 * 1e6) / (PULSES_PER_REVOLUTION * timeDeltaMicros);
  }

  lastTimeMicros = currentTimeMicros; // Update the last pulse time
  pulseCount += direction; // Increment or decrement pulse count based on direction
}
