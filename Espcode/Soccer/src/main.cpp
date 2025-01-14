#include "Remotexy.h"
#include <PID_v1.h>

struct{
    float Velx;
    float Vely;
    float w;
    int Servo;
}Joy;


//Pin Number Init
#define Motor1_Enc_CHA 32
#define Motor1_Enc_CHB 33
#define Motor2_Enc_CHA 34
#define Motor2_Enc_CHB 35
#define Motor3_Enc_CHA 36
#define Motor3_Enc_CHB 39
//Pin Number End
//Float Definition Starts
#define SQRT3_2 0.86602540378
#define HALF 0.5
//Float Definition Ends

SemaphoreHandle_t xSemaphore;

//Public Variable Delclaration Starts
float MotorRpm[3] = {0, 0, 0};
long prevT[3] = {0, 0, 0};
int Dir[3]={1,1,1}; 
float Motor_Vel[3]={0.0,0.0,0.0};
uint8_t MotorPWM[3]={0,0,0};
float Command_Rpm[3]={0.0,0.0,0.0};

     

//Public Varaible Declaration Ends


 //Function Declaration
void IRAM_ATTR updateMotorRPM1();
void IRAM_ATTR updateMotorRPM2();
void IRAM_ATTR updateMotorRPM3();
void Obtain_Normalize_JoyData(void *parameter);
void InvKinematics(void *parameter);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

 //Function Declaration Ends


void setup() {
    Serial.begin(9600);
    RemoteXY_Init(); 
    xSemaphore=xSemaphoreCreateMutex();
    if (xSemaphore != NULL) {
        xTaskCreatePinnedToCore(InvKinematics, "Task 1", 1000, NULL, 1, NULL,0);
        xTaskCreatePinnedToCore(Obtain_Normalize_JoyData, "Task 2", 4096, NULL, 1, NULL,1);
    } 
    else {
        Serial.println("Failed to create mutex.");
    }
//PinMode and interrupt Set Starts
    pinMode(Motor1_Enc_CHA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Motor1_Enc_CHA), updateMotorRPM1, RISING);
    pinMode(Motor2_Enc_CHA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Motor2_Enc_CHA), updateMotorRPM2, RISING);
    pinMode(Motor3_Enc_CHA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Motor3_Enc_CHA), updateMotorRPM3, RISING);
//PinMode and interrupt Set Ends
//PID INIT STARTS

//PID INIT STOPS
}

void loop() {

}

void updateMotorRPM1() {
    Dir[0] = (digitalRead(Motor1_Enc_CHB) > 0) ? 1 : -1;
    long currT = micros(); 
    MotorRpm[0] = Dir[0] / ((float)(currT - prevT[0])) / 6.0e5; 
    prevT[0] = currT; 
}

void updateMotorRPM2() {
    Dir[1] = (digitalRead(Motor2_Enc_CHB) > 0) ? 1 : -1;
    long currT = micros(); 
    MotorRpm[1] = Dir[1] / ((float)(currT - prevT[1])) / 6.0e5; 
    prevT[1] = currT; 
}

void updateMotorRPM3() {
    Dir[2] = (digitalRead(Motor3_Enc_CHB) > 0) ? 1 : -1;
    long currT = micros(); 
    MotorRpm[2] = Dir[2] / ((float)(currT - prevT[2])) / 6.0e5; 
    prevT[2] = currT; 
}

void InvKinematics(void *parameters){    
    while(true){
        if(xSemaphoreTake(xSemaphore,portMAX_DELAY)){
         Motor_Vel[0] = -Joy.Velx+(float)Joy.w;
         Motor_Vel[1] = HALF * Joy.Velx - SQRT3_2 * Joy.Vely +(float)Joy.w;
         Motor_Vel[2] = HALF * Joy.Velx + SQRT3_2 * Joy.Vely +(float)Joy.w; 
         Command_Rpm[0]=mapFloat(Motor_Vel[0],-1,1,-500,500);
         Command_Rpm[1]=mapFloat(Motor_Vel[1],-1,1,-500,500);
         Command_Rpm[2]=mapFloat(Motor_Vel[2],-1,1,-500,500);
        xSemaphoreGive(xSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
 }
void Obtain_Normalize_JoyData(void *parameter) {
    while (true) {
        RemoteXY_Handler();
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
            Joy.Velx = (float)RemoteXY.joystick_01_x / 200;
            Joy.Vely = (float)RemoteXY.joystick_01_y / 200;
            if(RemoteXY.button_05==1)
            Joy.w=0.3;
            else if(RemoteXY.button_06==1)
            Joy.w=-0.3;
            else
            Joy.w=0;
            xSemaphoreGive(xSemaphore);
            RemoteXY.value_01=Joy.Velx;
            RemoteXY.value_02=Joy.Vely;
            //Serial.print( RemoteXY.value_01);
           // Serial.print( RemoteXY.value_02);
           Serial.print(Command_Rpm[0],3);
           Serial.print(",");
           Serial.print(Command_Rpm[1],3);
           Serial.print(",");
           Serial.println(Command_Rpm[2],3);



        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}






























