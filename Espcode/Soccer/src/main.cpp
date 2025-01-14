#include "Remotexy.h"
#include <PID_v1.h>

struct{
    float Velx;
    float Vely;
    int w;
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
float Vel[3]={0.0,0.0,0.0};
//Public Varaible Declaration Ends


 //Function Declaration
void IRAM_ATTR updateMotorRPM1();
void IRAM_ATTR updateMotorRPM2();
void IRAM_ATTR updateMotorRPM3();
void Obtain_Normalize_JoyData(void *parameter);
void InvKinematics(void *parameter);
 //Function Declaration Ends


void setup() {
    Serial.begin(115200);
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
         Vel[0] = -Joy.Velx+Joy.w;
         Vel[1] = HALF * Joy.Velx - SQRT3_2 * Joy.Vely +Joy.w;
         Vel[2] = HALF * Joy.Velx + SQRT3_2 * Joy.Vely +Joy.w;   
        xSemaphoreGive(xSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
 }
void Obtain_Normalize_JoyData(void *parameter) {
    while (true) {
        RemoteXY_Handler();
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
            Joy.Velx = (float)RemoteXY.joystick_01_x / 100;
            Joy.Vely = (float)RemoteXY.joystick_01_y / 100;
            if(RemoteXY.button_05==1)
            Joy.w=1;
            else if(RemoteXY.button_06==1)
            Joy.w=-1;
            else
            Joy.w=0;
            xSemaphoreGive(xSemaphore);
            RemoteXY.value_01=Joy.Velx;
            RemoteXY.value_02=Joy.Vely;
            //Serial.print( RemoteXY.value_01);
           // Serial.print( RemoteXY.value_02);
           Serial.print(Vel[0],3);
           Serial.print(",");
           Serial.print(Vel[1],3);
           Serial.print(",");
           Serial.println(Vel[2],3);



        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}





























