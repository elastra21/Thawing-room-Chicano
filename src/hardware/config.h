#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>
#include "WS_V2.h"
#include <DallasTemperature.h>

// #define TIME_ZONE_OFFSET_HRS            (-7)  /* Ensenada, México */
#define TIME_ZONE_OFFSET_HRS            (+8)   /* Taiping, Malaysia */

// temperature acquisition filter 
#define HIGH_TEMP_LIMIT 60
#define LOW_TEMP_LIMIT -40

// setting PWM properties
#define AIR_PWM     7   
#define FREQ        5000 
#define AIR_PIN     PORT_C2  
#define RESOLUTION  6    

//------------ IO's    -------------------------------------------------------------------->
#define STAGE_1_IO  PORT_D0   
#define STAGE_2_IO  PORT_D1
#define STAGE_3_IO  PORT_D2
#define VALVE_IO    PORT_C3 // changed to have more logic on the board
#define FAN_IO      PORT_C0  
#define FAN2_IO     PORT_C1 
// #define STOP_IO     DI_0    
// #define DLY_S_IO    DI_1    
// #define START_IO    DI_2    

#define START_IO    PORT_B0  
#define DLY_S_IO    PORT_B1   
#define STOP_IO     PORT_B2

// #define A0    13 //ONE_WIRE_BUS  

#define TA_AI       AI_0
#define TS_AI       AI_1
#define TC_AI       AI_2

#define BUFFER_SIZE 60 

// Temperature Sensors settings          -------------------------------------------------------------------->
#define ONE_WIRE_BUS 13// Data wire is plugged into port 2 on the Arduin
#define TEMPERATURE_PRECISION 12

#define TIME_ACQ_DELAY 1000 //in ms the delay between temperature value refresh
#define AVG_RESOLUTION 1000   //in ms the sampling for the Ts measure

// DeviceAddress ADDRESS_TA = { 0x28, 0x8C, 0x4B, 0xAD, 0x27, 0x19, 0x01, 0xCA }; // Ta
// // DeviceAddress ADDRESS_TC1 = { 0x28, 0xA7, 0x93, 0x8B, 0x0B, 0x00, 0x00, 0xB2 }; // Ta
// DeviceAddress ADDRESS_TS = { 0x28, 0x78, 0x98, 0x8B, 0x0B, 0x00, 0x00, 0x22 }; // Ts
// DeviceAddress ADDRESS_TC = { 0x28, 0xDA, 0xB6, 0xF7, 0x3A, 0x19, 0x01, 0x85 }; // Tc 
// DeviceAddress ADDRESS_TI = { 0x28, 0x5A, 0xD3, 0x2A, 0x0D, 0x00, 0x00, 0x94 }; // Ti

#define IR_SENSOR_ADDRESS 0x5A
#define READ_TEMPERATURE 0x07

#endif
