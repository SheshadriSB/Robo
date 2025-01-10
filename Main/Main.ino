#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>

void handleGamepadInput(void *parameter);
void displayGamepadData(void *parameter);
void CalculateRpm(void *parameter);
void IRAM_ATTR updatePulseMotor1();
void IRAM_ATTR updatePulseMotor2();
void IRAM_ATTR updatePulseMotor3();

float MotorRpm[3]; 
volatile int pulseCount[3] = {0, 0, 0}; 
const int PPR = 210; 


const int Motor1_Pin = 16;
const int Motor2_Pin = 17;
const int Motor3_Pin = 18;

unsigned long lastCalculationTime[3] = {0, 0, 0}; 

struct JoyStick {
  float xPos = 0.0;
  float yPos = 0.0;
  float w=0.0;
  String keyPressed = "";
};
JoyStick Joy;

SemaphoreHandle_t xSemaphore;

void setup() {
  Serial.begin(115200);
  Dabble.begin("MyEsp32");
  xSemaphore = xSemaphoreCreateMutex();
  pinMode(Motor1_Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Motor1_Pin), updatePulseMotor1, RISING);
  pinMode(Motor2_Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Motor2_Pin), updatePulseMotor2, RISING);
  pinMode(Motor3_Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Motor3_Pin), updatePulseMotor3, RISING);
  xTaskCreatePinnedToCore(handleGamepadInput, "Gamepad Input", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(displayGamepadData, "Display Data", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(CalculateRpm, "Calculate RPM", 2048, NULL, 1, NULL, 1);
}

void loop() {
}
void IRAM_ATTR updatePulseMotor1() {
  pulseCount[0]++;
}
void IRAM_ATTR updatePulseMotor2() {
  pulseCount[1]++;
}
void IRAM_ATTR updatePulseMotor3() {
  pulseCount[2]++;
}
void CalculateRpm(void *parameter) {
  const unsigned long calculationInterval = 1000; 
  while (true) {
    unsigned long currentTime = millis();
    for (int i = 0; i < 3; i++) {
      if (currentTime - lastCalculationTime[i] >= calculationInterval){
        MotorRpm[i] = (pulseCount[i] * 60.0) / PPR;
        pulseCount[i] = 0;
        lastCalculationTime[i] = currentTime;
      }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
}

void handleGamepadInput(void *parameter) {
  while (true) {
    Dabble.processInput();

    if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
      Joy.keyPressed = "";

      if (GamePad.isUpPressed()) Joy.keyPressed += "Up ";
      if (GamePad.isDownPressed()) Joy.keyPressed += "Down ";
      if (GamePad.isLeftPressed()) Joy.keyPressed += "Left ";
      if (GamePad.isRightPressed()) Joy.keyPressed += "Right ";
      if (GamePad.isSquarePressed()) Joy.keyPressed += "Square ";
      if (GamePad.isCirclePressed()) Joy.keyPressed += "Circle ";
      if (GamePad.isCrossPressed()) Joy.keyPressed += "Cross ";
      if (GamePad.isTrianglePressed()) Joy.keyPressed += "Triangle ";
      if (GamePad.isStartPressed()) Joy.keyPressed += "Start ";
      if (GamePad.isSelectPressed()) Joy.keyPressed += "Select ";

      Joy.xPos = GamePad.getXaxisData();
      Joy.yPos = GamePad.getYaxisData();

      xSemaphoreGive(xSemaphore);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void displayGamepadData(void *parameter) {
  while (true) {

    Serial.print(Joy.xPos, 2);
    Serial.print(",");
    Serial.println(Joy.yPos, 2);


    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
