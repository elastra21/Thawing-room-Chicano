#ifndef LOGGER_H
#define LOGGER_H

#define WebSerial Serial
// #define SD_Logs true

#include "SD.h"
// #include <WS_V2.h>
#include <Arduino.h>

#define SCK   36                //SCK on SPI3
#define MOSI  35                //MISO on SPI3 
#define MISO  37                //MOSI on SPI3
#define SS    38

#ifdef WebSerial
  // No incluir WebSerialLite.h
#else
  #include "WebSerialLite.h"
#endif

class Logger {
private:
    #ifdef WebSerial_h
        #define outputStream WebSerial
    #else
        #define outputStream Serial
    #endif

    void setupSD();

public:
    void init(unsigned long baudRate);
    
    void print(const String &message);

    void println(const String &message);
    
    void printValue(const String &key, const String &value);

    void printTime(const String &prefix, int hour, int minute, int day, int month);
};

extern Logger logger; 

#endif // LOGGER_H