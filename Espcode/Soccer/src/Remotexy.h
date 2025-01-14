/*
   -- New project --
   
   This source code of graphical user interface 
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 3.1.13 or later version 
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/                   
     - for ANDROID 4.15.01 or later version;
     - for iOS 1.12.1 or later version;
    
   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.    
*/

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// you can enable debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG    

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__ESP32CORE_BLE

#include <BLEDevice.h>

// RemoteXY connection settings 
#define REMOTEXY_BLUETOOTH_NAME "RemoteXY"


#include <RemoteXY.h>

// RemoteXY GUI configuration  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 117 bytes
  { 255,8,0,0,0,110,0,19,0,0,0,0,180,2,108,200,200,80,1,1,
  7,0,5,8,5,65,65,7,23,55,55,32,2,26,7,1,8,146,18,18,
  143,56,16,16,0,2,31,83,0,1,56,147,18,18,125,39,16,16,0,2,
  31,65,0,1,32,125,18,18,160,39,16,16,0,2,31,68,0,1,32,168,
  18,18,143,22,16,16,0,2,31,87,0,1,80,162,24,24,23,245,21,21,
  8,2,31,0,1,82,6,24,24,166,245,22,22,8,2,31,0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  int8_t joystick_01_x; // from -100 to 100
  int8_t joystick_01_y; // from -100 to 100
  uint8_t button_01; // =1 if button pressed, else =0
  uint8_t button_02; // =1 if button pressed, else =0
  uint8_t button_03; // =1 if button pressed, else =0
  uint8_t button_04; // =1 if button pressed, else =0
  uint8_t button_05; // =1 if button pressed, else =0
  uint8_t button_06; // =1 if button pressed, else =0

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   
#pragma pack(pop)
 
/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////
