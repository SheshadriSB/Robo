#include "Remotexy.h"
#include <PID_v1.h>

// Joystick Structure
struct {
    int Velx;
    int Vely;
    int w;
    int Servo;
} Joy;

// Pin Number Definitions
#define Motor1_Enc_CHA 15
#define Motor1_Enc_CHB 36
#define Motor2_Enc_CHA 35
#define Motor2_Enc_CHB 32
#define Motor3_Enc_CHA 25
#define Motor3_Enc_CHB 26
#define Motor1_IN_1 4
#define Motor1_IN_2 16
#define Motor1_EN 17
#define Motor2_IN_1 19
#define Motor2_IN_2 18
#define Motor2_EN 21
#define Motor3_IN_1 12
#define Motor3_IN_2 14
#define Motor3_EN 13
// PWM Channel Configurations
#define PWM_FREQ 5000  // Frequency in Hz
#define PWM_RES 8      // Resolution (0–255 for 8-bit)
#define PWM_CHANNEL_1 0
#define PWM_CHANNEL_2 1
#define PWM_CHANNEL_3 2
// Constants
#define SQRT3_2 0.86602540378
#define HALF 0.5
#define Deg_to_Rad 0.01745329252
#define RPM_VEL_RATIO 3.351955
#define Ratiotofindrpm 285714.285714
SemaphoreHandle_t xSemaphore;

// Variables
volatile float MotorRpm[3] = {0, 0, 0};
volatile long prevT[3] = {0, 0, 0};
volatile int Dir[3] = {1, 1, 1};
bool Command_Dir[3]={false,false,false};
int Command_Motor_Vel[3] = {0, 0, 0};
int MotorPWM[3] = {0, 0, 0};
float Command_Rpm[3] = {0.0, 0.0, 0.0};

// Function Declarations
void IRAM_ATTR updateMotorRPM1();
void IRAM_ATTR updateMotorRPM2();
void IRAM_ATTR updateMotorRPM3();
void Obtain_Normalize_JoyData(void *parameter);
void InvKinematics(void *parameter);
void pinDeclaration();
void Move_Motor(void *parameter);
void PIDControl(void *parameter);
//PID DECLARATION

double Setpoint[3], Input[3], Output[3];
double Kp = 0.5, Ki = 1.0, Kd = 0.0;
PID myPID1(&Input[0], &Output[0], &Setpoint[0], Kp, Ki, Kd, DIRECT);
PID myPID2(&Input[1], &Output[1], &Setpoint[1], Kp, Ki, Kd, DIRECT);
PID myPID3(&Input[2], &Output[2], &Setpoint[2], Kp, Ki, Kd, DIRECT);

void setup() {
    Serial.begin(9600);
pinDeclaration();
    RemoteXY_Init();
//PID INIT
    // Initialize PID controllers
    myPID1.SetMode(AUTOMATIC);
    myPID2.SetMode(AUTOMATIC);
    myPID3.SetMode(AUTOMATIC);

    myPID1.SetOutputLimits(0, 255); // Motor PWM range
    myPID2.SetOutputLimits(0, 255);
    myPID3.SetOutputLimits(0, 255);

    xSemaphore = xSemaphoreCreateMutex();
    if (xSemaphore != NULL) {
        xTaskCreatePinnedToCore(InvKinematics, "InvKinematics", 1000, NULL, 1, NULL, 0);
        xTaskCreatePinnedToCore(Obtain_Normalize_JoyData, "NormalizeJoyData", 4096, NULL, 1, NULL, 1);
        xTaskCreatePinnedToCore(Move_Motor, "Motor", 4096, NULL, 1, NULL, 1);
        xTaskCreatePinnedToCore(PIDControl, "PID", 4096, NULL, 1, NULL, 1);
    } else {
        Serial.println("Failed to create mutex.");
    }
    // Pin Mode and Interrupt Setup
    pinMode(Motor1_Enc_CHA, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(Motor1_Enc_CHA), updateMotorRPM1, RISING);
    pinMode(Motor2_Enc_CHA, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(Motor2_Enc_CHA), updateMotorRPM2, RISING);
    pinMode(Motor3_Enc_CHA, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(Motor3_Enc_CHA), updateMotorRPM3, RISING);
}

void pinDeclaration(){
pinMode(Motor1_IN_1,OUTPUT);
pinMode(Motor1_IN_2,OUTPUT);
pinMode(Motor2_IN_1,OUTPUT);
pinMode(Motor2_IN_2,OUTPUT);
pinMode(Motor3_IN_1,OUTPUT);
pinMode(Motor3_IN_2,OUTPUT);
    ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RES);
    ledcAttachPin(Motor1_EN, PWM_CHANNEL_1);

    ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RES);
    ledcAttachPin(Motor2_EN, PWM_CHANNEL_2);

    ledcSetup(PWM_CHANNEL_3, PWM_FREQ, PWM_RES);
    ledcAttachPin(Motor3_EN, PWM_CHANNEL_3);
}

void loop() {
Serial.print(MotorRpm[0]);
Serial.print(",");
Serial.print(MotorRpm[1]);
Serial.print(",");
Serial.println(MotorRpm[2]);


}

void updateMotorRPM1() {
    long currT = micros();
    if (currT != prevT[0]) { // Avoid division by zero
        Dir[0] = (digitalRead(Motor1_Enc_CHB) == HIGH) ? 1 : -1;
        MotorRpm[0] =Ratiotofindrpm / (currT - prevT[0]);
        prevT[0] = currT;
    }
}

void updateMotorRPM2() {
    long currT = micros();
    if (currT != prevT[1]) { // Avoid division by zero
        Dir[1] = (digitalRead(Motor1_Enc_CHB) == HIGH) ? 1 : -1;
        MotorRpm[1] =Ratiotofindrpm / (currT - prevT[1]);
        prevT[1] = currT;
    }
}

void updateMotorRPM3() {
    long currT = micros();
    if (currT != prevT[2]) { // Avoid division by zero
        Dir[2] = (digitalRead(Motor1_Enc_CHB) == HIGH) ? 1 : -1;
        MotorRpm[2] =Ratiotofindrpm / (currT - prevT[2]);
        prevT[2] = currT;
    }
}
void InvKinematics(void *parameters) {
    while (true) {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
            Command_Motor_Vel[0] = -Joy.Velx + (Joy.w * Deg_to_Rad);
            Command_Motor_Vel[1] =( HALF * Joy.Velx) - (SQRT3_2 * Joy.Vely) + (Joy.w * Deg_to_Rad);
            Command_Motor_Vel[2] = (HALF * Joy.Velx )+ (SQRT3_2 * Joy.Vely) + (Joy.w * Deg_to_Rad);
            Command_Rpm[0]=RPM_VEL_RATIO*Command_Motor_Vel[0];
            Command_Rpm[1]=RPM_VEL_RATIO*Command_Motor_Vel[1];
            Command_Rpm[2]=RPM_VEL_RATIO*Command_Motor_Vel[2];
                    xSemaphoreGive(xSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void Obtain_Normalize_JoyData(void *parameter) {
    while (true) {
        RemoteXY_Handler();
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
            Joy.Velx = RemoteXY.joystick_01_x;
            Joy.Vely = RemoteXY.joystick_01_y;
            Joy.w = RemoteXY.button_05 ? 3000 : (RemoteXY.button_06 ? -3000 : 0);
            xSemaphoreGive(xSemaphore);
            RemoteXY.value_01 = Joy.Velx;
            RemoteXY.value_02 = Joy.Vely;

        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void Move_Motor(void *parameter) {
    while (true) {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
            // Motor 1 Direction
            if (Command_Rpm[0] > 0) {
                digitalWrite(Motor1_IN_1, HIGH);
                digitalWrite(Motor1_IN_2, LOW);
            } else if (Command_Rpm[0] < 0) {
                digitalWrite(Motor1_IN_1, LOW);
                digitalWrite(Motor1_IN_2, HIGH);
            } else { // Stop motor
                digitalWrite(Motor1_IN_1, LOW);
                digitalWrite(Motor1_IN_2, LOW);
            }

            // Motor 2 Direction
            if (Command_Rpm[1] > 0) {
                digitalWrite(Motor2_IN_1, HIGH);
                digitalWrite(Motor2_IN_2, LOW);
            } else if (Command_Rpm[1] < 0) {
                digitalWrite(Motor2_IN_1, LOW);
                digitalWrite(Motor2_IN_2, HIGH);
            } else {
                digitalWrite(Motor2_IN_1, LOW);
                digitalWrite(Motor2_IN_2, LOW);
            }

            // Motor 3 Direction
            if (Command_Rpm[2] > 0) {
                digitalWrite(Motor3_IN_1, HIGH);
                digitalWrite(Motor3_IN_2, LOW);
            } else if (Command_Rpm[2] < 0) {
                digitalWrite(Motor3_IN_1, LOW);
                digitalWrite(Motor3_IN_2, HIGH);
            } else {
                digitalWrite(Motor3_IN_1, LOW);
                digitalWrite(Motor3_IN_2, LOW);
            }
            // Write PWM
    ledcWrite(PWM_CHANNEL_1, MotorPWM[0]);
    ledcWrite(PWM_CHANNEL_2, MotorPWM[1]);
    ledcWrite(PWM_CHANNEL_3, MotorPWM[2]);

            xSemaphoreGive(xSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void PIDControl(void *parameter) {
    while (true) {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
            // Update PID Inputs
            Input[0] = MotorRpm[0];
            Input[1] = MotorRpm[1];
            Input[2] = MotorRpm[2];

            // Update PID Setpoints
            Setpoint[0] = abs(Command_Rpm[0]);
            Setpoint[1] = abs(Command_Rpm[1]);
            Setpoint[2] = abs(Command_Rpm[2]);
            //OUTPUT
            MotorPWM[0]=Output[0];
            MotorPWM[1]=Output[1];
            MotorPWM[2]=Output[2];
            // Compute PID Outputs
            myPID1.Compute();
            myPID2.Compute();
            myPID3.Compute();



            xSemaphoreGive(xSemaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Adjust as needed
    }
}

