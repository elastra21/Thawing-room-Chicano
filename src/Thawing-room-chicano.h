#ifndef THAWING_ROOM_H
#define THAWING_ROOM_H

#include <Wire.h>
#include "Stage.h"
#include <PID_v1.h>
// #include "secrets.h"
#include <Arduino.h>
#include "MqttClient.h"
#include "hardware/Logger.h"
#include "hardware/config.h"
#include "hardware/Controller.h"

//------------ structure definitions an flags -------------------------------------------------------->

// temperature measures
typedef struct { float ta; float ts; float tc; float ti; float avg_ts; } data_s;


enum SystemState {
    IDLE,
    STAGE1,
    STAGE2,
    STAGE3,
    ERROR
};

enum SensorProbes{TA_TYPE, TS_TYPE, TC_TYPE};
enum button_type{NONE, D_START, START, STOP};

//---- Function declaration ----/////////////////////////////////////////////////////////////////////////////

void handleStage();
void setStage(SystemState Stage);

void stopRoutine();

void initStage1();
void initStage2();
void initStage3();

void handleStage1();
void handleStage2();
void handleStage3();

void idle();

void asyncLoopSprinkler();

void getTsAvg();
void updateTemperature();
bool handleInputs(button_type override = NONE);
void callback(char *topic, byte *payload, unsigned int len); 
bool hasIntervalPassed(uint32_t &previousMillis, uint32_t interval, bool to_min = false);
bool isValidTemperature(float temp, float minTemp, float maxTemp, const String& sensorName);

void publishPID();
void onMQTTConnect();
void aknowledgementRoutine();
void publishTemperatures(DateTime &current_date);
void publishStateChange(const char* topic, int state, const String& message);


// void sendTemperaturaAlert(float temp, String sensor);


//---- timing settings -----////////////////////////////////////////////////////////////////////////////////

#define MINS 60000

// ---- Probes min and max values ----//////////////////////////////////////////////////////////////////////// 

#define TA_MIN -5
#define TA_MAX 25
#define TA_DEF 15 

#define TS_MIN -20
#define TS_MAX 10
#define TS_DEF 5

#define TC_MIN -20
#define TC_MAX 5
#define TC_DEF -1


#endif 

/*
DATA_FOLER:
- data
---- config.txt
---- config_ENSENADA.txt
---- config_TAIPING.txt
---- defaultParameters.txt
---- log.txt

- src
---- THAWING-ROOM-CHICANO

// Main processes
-------- handleStage()
-------- setStage(stage)
-------- getStep(stage) // This might be not a good idea

// Sub-processes
-------- stage1(step)
-------- stage2(step)
-------- stage3(step)

// Helpers
-------- hasIntervalPassed(uint32_t &previousMillis, uint32_t interval, bool to_min = false)
-------- updateTemperatures()
-------- handleInputs(button_type override)
-------- isValidTemperature(float temp, float minTemp, float maxTemp, const String& sensorName)
-------- void getTsAvg();


// Communication
-------- publishStateChange(const char* topic, int state, const String& message)
-------- publishTemperatures(DateTime &current_date)
-------- aknowledgementRoutine()
-------- publishPID()
*/