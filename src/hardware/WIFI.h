#ifndef MY_WIFI_H
#define MY_WIFI_H
#include "EEPROM.h"
#include "SPIFFS.h"
#include <Update.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <WiFiMulti.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>

#ifdef WebSerial
  // No incluir WebSerialLite.h
#else
  #include "WebSerialLite.h"
#endif
#include "ESPAsyncWebServer.h"

#define SSID_SIZE 32
#define PASSWORD_SIZE 64
#define HOSTNAME_SIZE 32
#define IP_ADDRESS_SIZE 16

class WIFI {
  public:
    void init(const char* ssid, const char* password, const char* hostname);
    void loopOTA();
    void setUpOTA();
    void reconnect();
    bool isConnected();
    void connectToWiFi();
    bool refreshWiFiStatus();
    bool getConnectionStatus();
    void setUpWebServer(bool brigeSerial = false);

    void DEBUG(const char *message);
  private:
    char ssid[SSID_SIZE];  
    char password[PASSWORD_SIZE];
    char hostname[HOSTNAME_SIZE]; 
    bool last_connection_state = false;
};
#endif
