#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "WS_V2.h"
#include "WIFI.h"
#include <Wire.h>
#include "config.h"
#include <RTClib.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include <OneWire.h>
#include <NTPClient.h>
#include <Preferences.h>
#include "SensorBuffer.h"
#include <DallasTemperature.h>

// #define WebSerial Serial

#define TEMPERATURE_MIN  -50 // Minimum temperature value (in Celsius)
#define TEMPERATURE_MAX  150
#define ADC__RESOLUTION  4095 
#define REFERENCE 3.3

#define SECS_IN_HR 3600

// #define TIME_ZONE_OFFSET_HRS            (-7)  /* Ensenada, México */
#define TIME_ZONE_OFFSET_HRS            (+8)   /* Taiping, Malaysia */

enum SystemState {
    IDLE,
    STAGE1,
    STAGE2,
    STAGE3,
    ERROR,
    NUM_STATES
};

struct StageState {
    uint8_t stage;
    uint8_t step;
};

class Controller {
private:
    WIFI wifi;
    RTC_DS3231 rtc;
    Preferences preferences;

    // Pinout pinout;
    // Logger logger;

    void setUpI2C();
    void setUpIOS();
    void setUpLogger();
    void setUpAnalogInputs();
    void setUpAnalogOutputs();
    void setUpDigitalInputs();
    void setUpDigitalOutputs();
public:
    ~Controller();
    Controller(/* args */);

    StageState getLastState();
    void saveLastState(StageState current_state);
    
    DeviceAddress ADDRESS_TA = { 0x28, 0x8C, 0x4B, 0xAD, 0x27, 0x19, 0x01, 0xCA }; // Ta
    DeviceAddress ADDRESS_TS = { 0x28, 0x78, 0x98, 0x8B, 0x0B, 0x00, 0x00, 0x22 }; // Ts
    DeviceAddress ADDRESS_TC = { 0x28, 0xDA, 0xB6, 0xF7, 0x3A, 0x19, 0x01, 0x85 }; // Tc 
    DeviceAddress ADDRESS_TI = { 0x28, 0x5A, 0xD3, 0x2A, 0x0D, 0x00, 0x00, 0x94 }; // Ti

    void init();
    void setUpRTC();
    bool isRTCConnected();
    DateTime getDateTime();
    void setUpOneWireProbes(); // -----> NOT DEFINED YET
    void updateProbesTemperatures(); // ----> NOT DEFINED YET
    float readTempFrom(uint8_t channel);
    bool readDigitalInput(uint8_t input);
    uint64_t readAnalogInput(uint8_t input);
    float getOneWireTempFrom(DeviceAddress address); // ----> NOT DEFINED YET
    void writeAnalogOutput(uint8_t output, uint8_t value);
    void writeDigitalOutput(uint8_t output, uint8_t value);
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

    // Logger
    void DEBUG(const char *message);

};

#endif