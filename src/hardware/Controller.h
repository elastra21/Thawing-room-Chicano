#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <FS.h>
#include "WIFI.h"
#include <Wire.h>
#include "MqttClient.h"
#include <WS_V2.h>
#include "config.h"
#include "Logger.h"
#include <SPIFFS.h>
#include <RTClib.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include <OneWire.h>
#include <NTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "SensorBuffer.h"
#include <Adafruit_MLX90640.h>
#include <DallasTemperature.h>

#define TEMPERATURE_MIN  -50 // Minimum temperature value (in Celsius)
#define TEMPERATURE_MAX  150
#define ADC__RESOLUTION  4095 
#define REFERENCE 3.3

#define SECS_IN_HR 3600

//Defining messages of ERRORS
#define I2C_ERR_TXT "No found I2C device"
#define RTC_ERR_TXT "No found RTC"
#define IR_ERR_TXT "No found MLX90640 Sensor"
#define TC_ERR_TXT "Tc readings out of range"
#define TS_ERR_TXT "Ts readings out of range"
#define TA_ERR_TXT "Ta readings out of range"
#define NO_CONFIG_FILE_TXT "No config file found"
#define NO_DEFAULT_PARAMS_TXT "No default parameters found"



// #define TIME_ZONE_OFFSET_HRS            (-7)  /* Ensenada, México */
// #define TIME_ZONE_OFFSET_HRS            (+8)   /* Taiping, Malaysia */

typedef struct {
    float fanOnTime;
    float fanOffTime;
    float sprinklerOnTime;  // Optional, can be 0 or not used for Stage 1
    float sprinklerOffTime; // Optional, can be 0 or not used for Stage 1
} stage_parameters;

enum SystemState {
    IDLE,
    STAGE1,
    STAGE2,
    STAGE3,
    ERROR,
    NUM_STATES
};

struct StageState {
    SystemState stage;
    uint8_t step;
};

// A and B variables
typedef struct { float A; float B; }                  room_parameters;

// Ts and Tc target value
typedef struct { float ts; float tc; }        data_tset;

class Controller {
private:
    enum ErrorType {
        I2C_ERROR,
        RTC_NOT_FOND,
        IR_NOT_FOUND,
        TC_OUT_OF_RANGE,
        TS_OUT_OF_RANGE,
        TA_OUT_OF_RANGE,
        NO_CONFIG_FILE,
        NO_DEFAULT_PARAMS,
        NUM_ERRORS
    };

    const char* ERROR_MESSAGES[NUM_ERRORS] = {
        I2C_ERR_TXT,
        RTC_ERR_TXT,
        IR_ERR_TXT,
        TC_ERR_TXT,
        TS_ERR_TXT,
        TA_ERR_TXT,
        NO_CONFIG_FILE_TXT,
        NO_DEFAULT_PARAMS_TXT
    };

    WIFI wifi;
    bool fan_state = false;
    int ARRAY_SIZE = 7;
    bool ir_ts = false;
    bool lora_tc = false;
    Preferences preferences;
    int8_t TIME_ZONE_OFFSET_HRS = 0;

    void setUpI2C();
    void setUpIOS();
    void setUpLogger();
    void setUpAnalogInputs();
    void setUpAnalogOutputs();
    void setUpDigitalInputs();
    void setUpDigitalOutputs();

//  IR Tc Stuff
    void setUpIRTc();
    float getMinTemp(float *temps);
    float getMaxTemp(float *temps);
    float getAvgBottomTemp(float *temps);
    void checkAndInsertBottomTemps(float temp, float *temps);

public:
    ~Controller();
    Controller(/* args */);

    bool thresLastState();
    StageState getLastState();
    void saveLastState(StageState current_state);
    
    DeviceAddress ADDRESS_TA = { 0x28, 0x8C, 0x4B, 0xAD, 0x27, 0x19, 0x01, 0xCA }; // Ta
    DeviceAddress ADDRESS_TS = { 0x28, 0x78, 0x98, 0x8B, 0x0B, 0x00, 0x00, 0x22 }; // Ts
    DeviceAddress ADDRESS_TC = { 0x28, 0xDA, 0xB6, 0xF7, 0x3A, 0x19, 0x01, 0x85 }; // Tc 
    DeviceAddress ADDRESS_TI = { 0x28, 0x5A, 0xD3, 0x2A, 0x0D, 0x00, 0x00, 0x94 }; // Ti

    void init();
    void setUpRTC();
    bool isLoraTc();
    float getIRTemp();
    bool getFanState();
    bool isRTCConnected();
    bool isTsContactLess();
    DateTime getDateTime();
    void setUpOneWireProbes(); // -----> NOT DEFINED YET
    void setLoraTc(bool value);
    void saveLogToSD(const String &message);
    void updateProbesTemperatures(); // ----> NOT DEFINED YET
    void setTsContactLess(bool value);
    float readTempFrom(uint8_t channel);
    bool readDigitalInput(uint8_t input);
    uint64_t readAnalogInput(uint8_t input);
    float getOneWireTempFrom(DeviceAddress address); // ----> NOT DEFINED YET
    void writeAnalogOutput(uint8_t output, uint8_t value);
    void writeDigitalOutput(uint8_t output, uint8_t value);
    String jsonBuilder(String keys[], float values[], int length);
    void turnOnFan(bool value, bool CCW = false);
    // WIFI CLASS
    void loopOTA();
    void WiFiLoop();
    void reconnectWiFi();
    bool isWiFiConnected();
    bool refreshWiFiStatus();
    bool getConnectionStatus();
    // Puto el que lo lea
    void connectToWiFi(bool web_server, bool web_serial, bool OTA); 
    void setUpWiFi(const char* ssid, const char* password, const char* hostname);
    void runConfigFile(char* ssid, char* password, char* hostname, char* ip_address, uint16_t* port, char* mqtt_id, char* username, char* mqtt_password, char* prefix_topic);
    void setUpDefaultParameters(stage_parameters &stage1_params, stage_parameters &stage2_params, stage_parameters &stage3_params, room_parameters &room, data_tset &N_tset);
    void updateDefaultParameters(stage_parameters &stage1_params, stage_parameters &stage2_params, stage_parameters &stage3_params, room_parameters &room, data_tset &N_tset);

    //Logger
    void DEBUG(const char *message);
    void ERROR(ErrorType error);
    // void DEBUG(String message);

    template <typename T> 
    bool updateConfigJson(const char* param, T value, String file = "/config.txt"){
        // Abre el archivo de configuración existente
        File configFile = SD.open(file, FILE_READ);
        if (!configFile) {
            DEBUG("Error al abrir el archivo de configuración para lectura");
            return false;
        }

        // Parsea el objeto JSON del archivo
        StaticJsonDocument<1024> doc; // Cambiado a StaticJsonDocument
        auto error = deserializeJson(doc, configFile);
        if (error) {
            Serial.println("Error al parsear el archivo de configuración");
            return false;
        }

        // Update the values
        doc[param] = value;

        // Open file for writing
        configFile = SD.open(file, FILE_WRITE);
        if (!configFile) {
            DEBUG("Error al abrir el archivo de configuración para escritura");
            return false;
        }

        // Serializa el JSON al archivo
        if (serializeJson(doc, configFile) == 0) {
            DEBUG("Error al escribir en el archivo de configuración");
            return false;
        }

        configFile.close();
        return true;
    }

};

#endif