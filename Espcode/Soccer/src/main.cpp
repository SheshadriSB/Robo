#define REMOTEXY_MODE__ESP32CORE_BLE
#define REMOTEXY_BLUETOOTH_NAME "RemoteXY"
#include <BLEDevice.h>
#include <RemoteXY.h>

#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] = {
    255, 10, 0, 0, 0, 96, 0, 19, 0, 0, 0, 0, 31, 1, 106, 200, 2, 1, 0, 8,
    0, 5, 3, 6, 65, 65, 32, 2, 26, 7, 1, 8, 146, 18, 18, 0, 2, 31, 0, 1,
    55, 147, 18, 18, 0, 2, 31, 0, 1, 31, 125, 18, 18, 0, 2, 31, 0, 1, 31, 168,
    18, 18, 0, 2, 31, 0, 1, 78, 162, 24, 24, 8, 2, 31, 0, 1, 80, 6, 24, 24,
    8, 2, 31, 0, 3, 85, 73, 16, 44, 3, 2, 26, 1, 0, 1, 43, 83, 24, 24, 0,
    2, 31, 0
};
#pragma pack(pop)

struct {
    int8_t joystick_01_x; // from -100 to 100
    int8_t joystick_01_y; // from -100 to 100
    uint8_t button_01;    // =1 if button pressed, else =0
    uint8_t button_02;    // =1 if button pressed, else =0
    uint8_t button_03;    // =1 if button pressed, else =0
    uint8_t button_04;    // =1 if button pressed, else =0
    uint8_t button_05;    // =1 if button pressed, else =0
    uint8_t button_06;    // =1 if button pressed, else =0
    uint8_t selectorSwitch_01; // from 0 to 3
    uint8_t button_07;    // =1 if button pressed, else =0
    uint8_t connect_flag; // =1 if wire connected, else =0
} RemoteXY;

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

SemaphoreHandle_t xSemaphore;

volatile int pulseCount[3] = {0, 0, 0};
float MotorRpm[3] = {0, 0, 0};
const int PPR = 210; 
unsigned long lastCalculationTime[3] = {0, 0, 0};

void IRAM_ATTR updatePulseMotor1();
void IRAM_ATTR updatePulseMotor2();
void IRAM_ATTR updatePulseMotor3();
void calculateRPMTask(void *parameter);
void displayGamepadDataTask(void *parameter);
void UpdateMotor(void *parameter);
void setup() {
    Serial.begin(115200);

  RemoteXY_Init (); 
    xSemaphore = xSemaphoreCreateMutex();

    pinMode(Motor1_Enc_CHA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Motor1_Enc_CHA), updatePulseMotor1, RISING);

    pinMode(Motor2_Enc_CHA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Motor2_Enc_CHA), updatePulseMotor2, RISING);

    pinMode(Motor3_Enc_CHA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Motor3_Enc_CHA), updatePulseMotor3, RISING);

    xTaskCreatePinnedToCore(displayGamepadDataTask, "DisplayGamepadData", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(calculateRPMTask, "CalculateRPM", 2048, NULL, 1, NULL, 1);
}

void loop() {
    RemoteXY_Handler();
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

void calculateRPMTask(void *parameter) {
    const unsigned long calculationInterval = 100; 
    while (true) {
        unsigned long currentTime = millis();
        for (int i = 0; i < 3; i++) {
            if (currentTime - lastCalculationTime[i] >= calculationInterval) {
                if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
                    MotorRpm[i] = (pulseCount[i] * 600.0) / PPR; 
                    pulseCount[i] = 0; 
                    lastCalculationTime[i] = currentTime; 
                    xSemaphoreGive(xSemaphore);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(calculationInterval)); 
    }
}

void displayGamepadDataTask(void *parameter) {
    while (true) {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
            Serial.print("Joystick X: ");
            Serial.println(RemoteXY.joystick_01_x);
            xSemaphoreGive(xSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
