#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>

#define Motor1_Enc_CHA 32
#define Motor1_Enc_CHB 33
#define Motor2_Enc_CHA 34
#define Motor2_Enc_CHB 35
#define Motor3_Enc_CHA 36
#define Motor4_Enc_CHB 39

#define Motor1_In1 13
#define Motor1_In2 14
#define Motor1_En 15

#define Motor2_In1 16
#define Motor2_In2 17
#define Motor2_En 18

#define Motor3_In1 19
#define Motor3_In2 21
#define Motor3_En 22
void handleGamepadInput(void *parameter);
void displayGamepadData(void *parameter);
void CalculateRpm(void *parameter);
void UpdateMotor(void *parameter);
void IRAM_ATTR updatePulseMotor1();
void IRAM_ATTR updatePulseMotor2();
void IRAM_ATTR updatePulseMotor3();
float normalize(float value, float minInput, float maxInput) {
    return (value - minInput) / (maxInput - minInput) * 2 - 1;
}
float MotorRpm[3]={0,0,0}; 
float Motow_Vel[3]={0,0,0};
volatile int pulseCount[3] = {0, 0, 0}; 
const int PPR = 210; 
unsigned long lastCalculationTime[3] = {0, 0, 0}; 
struct JoyStick {
  float Angle = 0.0;
  float Radius = 0.0;
  float w=0.0;
  String D_Pressed = "";
  String Key_Pressed="";
  String Special_Pressed="";
};
JoyStick Joy;
SemaphoreHandle_t xSemaphore;

void setup() {
  Serial.begin(115200);
  Dabble.begin("MyEsp32");
  xSemaphore = xSemaphoreCreateMutex();
  pinMode(Motor1_Enc_CHA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Motor1_Enc_CHA), updatePulseMotor1, RISING);
  pinMode(Motor2_Enc_CHA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Motor2_Enc_CHA), updatePulseMotor2, RISING);
  pinMode(Motor3_Enc_CHA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Motor3_Enc_CHA), updatePulseMotor3, RISING);
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
      Joy.Key_Pressed = "";
      Joy.D_Pressed="";
      Joy.Special_Pressed="";
      Joy.w=0;

      if (GamePad.isUpPressed()) Joy.D_Pressed += "Up ";
      if (GamePad.isDownPressed()) Joy.D_Pressed += "Down ";
      if (GamePad.isLeftPressed()) Joy.D_Pressed += "Left ";
      if (GamePad.isRightPressed()) Joy.D_Pressed += "Right ";
      if (GamePad.isSquarePressed()) Joy.Key_Pressed += "Square ", Joy.w=-3;
      if (GamePad.isCirclePressed()) Joy.Key_Pressed += "Circle " , Joy.w=3;
      if (GamePad.isCrossPressed()) Joy.Key_Pressed += "Cross ";
      if (GamePad.isTrianglePressed()) Joy.Key_Pressed += "Triangle ";
      if (GamePad.isStartPressed()) Joy.Special_Pressed += "Start ";
      if (GamePad.isSelectPressed()) Joy.Special_Pressed += "Select ";

      Joy.Angle = GamePad.getAngle();
      Joy.Radius = GamePad.getRadius();
      xSemaphoreGive(xSemaphore);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void displayGamepadData(void *parameter) {
  while (true) {

    Serial.print(Joy.Angle, 2);
    Serial.print(",");
    Serial.println(Joy.Radius, 2);



    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
void UpdateMotor(void *parameter){
  
}

