#ifndef THAWING_ROOM_H
#define THAWING_ROOM_H

#include <Wire.h>
#include "Stage.h"
#include <PID_v1.h>
#include <Button.h>
#include <Arduino.h>
#include "MqttClient.h"
#include <TaskScheduler.h>
#include "./hardware/Logger.h"
#include "config.h"
#include "./hardware/Controller.h"

//------------ structure definitions an flags -------------------------------------------------------->

// temperature measures
typedef struct { float ta; float ts; float tc; float ti; float avg_ts; float avg_tc; float avg_ta; } data_s;


const int stageLedPins[NUM_STATES] = {
    -1, // IDLE no tiene LED asociado
    STAGE_1_IO,
    STAGE_2_IO,
    STAGE_3_IO,
    -1  // ERROR no tiene LED asociado, o puedes asignar un pin si hay un LED para ERROR
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

void destroyStage1();
void destroyStage2();
void destroyStage3();

void idle();

void asyncLoopSprinkler(uint32_t &timer, uint32_t offTime, uint32_t onTime);

void getTempAvg();
void updateTemperature();
bool handleInputs(button_type override = NONE);
void callback(char *topic, byte *payload, unsigned int len); 
bool hasIntervalPassed(uint32_t &previousMillis, uint32_t interval, bool to_min = false);
bool isValidTemperature(float temp, float minTemp, float maxTemp, const String& sensorName);

void publishPID();
void onMQTTConnect();
void aknowledgementRoutine();
void publishTemperatures(DateTime &current_date);
void publishTemperatures();
void publishStateChange(const char* topic, int state, const String& message);


void sendTemperaturaAlert(float temp, String sensor);
void turn_on_flush_routine();
void turn_off_flush_routine();


//---- timing settings -----////////////////////////////////////////////////////////////////////////////////

#define MINS 60000

// ---- Probes min and max values ----//////////////////////////////////////////////////////////////////////// 

#define TA_MIN -9999
#define TA_MAX 9999
#define TA_DEF 15 

#define TS_MIN -9999
#define TS_MAX 9999
#define TS_DEF 5

#define TC_MIN -9999
#define TC_MAX 9999
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