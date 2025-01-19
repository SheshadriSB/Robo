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
#define Motor1_Enc_CHA 27
#define Motor1_Enc_CHB 15
#define Motor2_Enc_CHA  35
#define Motor2_Enc_CHB  32
#define Motor3_Enc_CHA  25
#define Motor3_Enc_CHB  26
#define Motor1_IN_1  18
#define Motor1_IN_2  19
#define Motor1_EN  21
#define Motor2_IN_1  16
#define Motor2_IN_2  4
#define Motor2_EN  17
#define Motor3_IN_1 12
#define Motor3_IN_2 14
#define Motor3_EN 13
// PWM Channel Configurations
#define PWM_FREQ 7000  // Frequency in Hz
#define PWM_RES 8      // Resolution (0–255 for 8-bit)
#define PWM_CHANNEL_1 0
#define PWM_CHANNEL_2 1
#define PWM_CHANNEL_3 2
// Constants
#define SQRT3_2 0.86602540378
#define HALF 0.5
#define Deg_to_Rad 0.01745329252
#define RPM_VEL_RATIO 3.351955
#define Ratiotofindrpm 142857.142857
SemaphoreHandle_t xSemaphore;
// Variables
int MotorPWM[3] = {0, 0, 0};
float Command_Rpm[3] = {0.0, 0.0, 0.0};
volatile float MotorRpm[3] = {0, 0, 0};
long prevT[3]={0,0,0};
int Dir[3]={0,0,0};
long int PulseCount[3]={0,0,0};
// Function Declarations
void IRAM_ATTR updateMotorRPM1();
void IRAM_ATTR updateMotorRPM2();
void IRAM_ATTR updateMotorRPM3();
void Obtain_Normalize_JoyData();
void InvKinematics();
void pinDeclaration();
void Move_Motor();
void PIDControl();
//PID DECLARATION
double Setpoint[3], Input[3], Output[3];
double Kp[3] ={ 0.6,0.6,0.6};
double Ki[3] = {2.0,2.0,2.0};
double Kd[3] = {0.0,0.0,0.0};
PID myPID1(&Input[0], &Output[0], &Setpoint[0], Kp[0], Ki[0], Kd[0], DIRECT);
PID myPID2(&Input[1], &Output[1], &Setpoint[1], Kp[1], Ki[1], Kd[1], DIRECT);
PID myPID3(&Input[2], &Output[2], &Setpoint[2], Kp[2], Ki[2], Kd[2], DIRECT);

void setup() {
    Serial.begin(9600);
pinDeclaration();
    RemoteXY_Init();
//PID INIT
    // Initialize PID controllers
    myPID1.SetMode(AUTOMATIC);
    myPID2.SetMode(AUTOMATIC);
    myPID3.SetMode(AUTOMATIC);

    myPID1.SetOutputLimits(-255, 255,-40,40); // Motor PWM range
    myPID2.SetOutputLimits(-255, 255,-40,40);
    myPID3.SetOutputLimits(-255, 255,-40,40);

    xSemaphore = xSemaphoreCreateMutex();
    if (xSemaphore != NULL) {
    } else {
        Serial.println("Failed to create mutex.");
    }

    // Pin Mode and Interrupt Setup
    pinMode(Motor1_Enc_CHA, INPUT_PULLUP);
    pinMode(Motor1_Enc_CHB, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(Motor1_Enc_CHA), updateMotorRPM1, CHANGE);
    pinMode(Motor2_Enc_CHA, INPUT_PULLUP);
    pinMode(Motor2_Enc_CHB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Motor2_Enc_CHA), updateMotorRPM2, CHANGE);
    pinMode(Motor3_Enc_CHA, INPUT_PULLUP);
    pinMode(Motor3_Enc_CHB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Motor3_Enc_CHA), updateMotorRPM3, CHANGE);
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
            RemoteXY_Handler();
            Obtain_Normalize_JoyData();
            InvKinematics();
            PIDControl();
            Move_Motor();
}

void updateMotorRPM1() {
  unsigned long currT = micros(); 
  int stateA = digitalRead(Motor1_Enc_CHA);
  int stateB = digitalRead(Motor1_Enc_CHB);
  if (stateA == stateB) 
    Dir[0] = 1; 
   else 
    Dir[0] = -1; 
  if (currT > 0) 
    MotorRpm[0] =Dir[0]*Ratiotofindrpm / (currT - prevT[0]);
  prevT[0] = currT; 
  PulseCount[0] += Dir[0]; 
}
void updateMotorRPM2() {
  unsigned long currT = micros();
  int stateA = digitalRead(Motor2_Enc_CHA);
  int stateB = digitalRead(Motor2_Enc_CHB);
  if (stateA == stateB) 
    Dir[1] = 1;
   else 
    Dir[1] = -1;
  if (currT > prevT[1]) 
    MotorRpm[1] = Dir[1] * Ratiotofindrpm / (currT - prevT[1]);
  prevT[1] = currT;
  PulseCount[1] += Dir[1];
}

void updateMotorRPM3() {
  unsigned long currT = micros();
  int stateA = digitalRead(Motor3_Enc_CHA);
  int stateB = digitalRead(Motor3_Enc_CHB);
  if (stateA == stateB) 
    Dir[2] = 1;
   else 
    Dir[2] = -1;
  if (currT > prevT[2]) 
    MotorRpm[2] = Dir[2] * Ratiotofindrpm / (currT - prevT[2]);
  prevT[2] = currT;
  PulseCount[2] += Dir[2];
}
void InvKinematics() {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
            Command_Rpm[0] = -Joy.Velx + (Joy.w * Deg_to_Rad);
            Command_Rpm[1] =( HALF * Joy.Velx) - (SQRT3_2 * Joy.Vely) + (Joy.w * Deg_to_Rad);
            Command_Rpm[2] = (HALF * Joy.Velx )+ (SQRT3_2 * Joy.Vely) + (Joy.w * Deg_to_Rad);
            Command_Rpm[0]*=RPM_VEL_RATIO;
            Command_Rpm[1]*=RPM_VEL_RATIO;
            Command_Rpm[2]*=RPM_VEL_RATIO;
                    xSemaphoreGive(xSemaphore);
        }
    
}
void Obtain_Normalize_JoyData() {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
            Joy.Velx = RemoteXY.joystick_01_x;
            Joy.Vely = -RemoteXY.joystick_01_y;
            Joy.w = RemoteXY.button_05 ? -3000 : (RemoteXY.button_06 ? 3000 : 0);
            xSemaphoreGive(xSemaphore);
            RemoteXY.value_01 = Joy.Velx;
            RemoteXY.value_02 = Joy.Vely;   
    }
}
void Move_Motor() {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
            // Motor 1 Direction
            if (MotorPWM[0] > 41) {
                digitalWrite(Motor1_IN_1, HIGH);
                digitalWrite(Motor1_IN_2, LOW);
            } else if (MotorPWM[0] < -41) {
                digitalWrite(Motor1_IN_1, LOW);
                digitalWrite(Motor1_IN_2, HIGH);
            } else { // Stop motor
                digitalWrite(Motor1_IN_1, LOW);
                digitalWrite(Motor1_IN_2, LOW);
            }

            // Motor 2 Direction
            if (MotorPWM[1] > 41) {
                digitalWrite(Motor2_IN_1, HIGH);
                digitalWrite(Motor2_IN_2, LOW);
            } else if (MotorPWM[1] < -41) {
                digitalWrite(Motor2_IN_1, LOW);
                digitalWrite(Motor2_IN_2, HIGH);
            } else {
                digitalWrite(Motor2_IN_1, LOW);
                digitalWrite(Motor2_IN_2, LOW);
            }

            // Motor 3 Direction
            if (MotorPWM[2] > 41) {
                digitalWrite(Motor3_IN_1, HIGH);
                digitalWrite(Motor3_IN_2, LOW);
            } else if (MotorPWM[2] < -41) {
                digitalWrite(Motor3_IN_1, LOW);
                digitalWrite(Motor3_IN_2, HIGH);
            } else {
                digitalWrite(Motor3_IN_1, LOW);
                digitalWrite(Motor3_IN_2, LOW);
            }
            // Write PWM
    ledcWrite(PWM_CHANNEL_1, abs(MotorPWM[0]));
    ledcWrite(PWM_CHANNEL_2, abs(MotorPWM[1]));
    ledcWrite(PWM_CHANNEL_3, abs(MotorPWM[2]));

            xSemaphoreGive(xSemaphore);
        
    }
}
void PIDControl() {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
            // Update PID Inputs
            Input[0] = MotorRpm[0];
            Input[1] = MotorRpm[1];
            Input[2] = MotorRpm[2];
            // Update PID Setpoints
            Setpoint[0] = Command_Rpm[0];
            Setpoint[1] = Command_Rpm[1];
            Setpoint[2] = Command_Rpm[2];

            // Compute PID Outputs
            myPID1.Compute();
            myPID2.Compute();
            myPID3.Compute();
            //OUTPUT
            MotorPWM[0]=Output[0];
            MotorPWM[1]=Output[1];
            MotorPWM[2]=Output[2];


            xSemaphoreGive(xSemaphore);
        }

    }


