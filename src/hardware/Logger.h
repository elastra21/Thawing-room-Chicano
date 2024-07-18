#ifndef LOGGER_H
#define LOGGER_H

#include "SD.h"
#include <RTClib.h> 
#include <Arduino.h>

#define SCK   36                //SCK on SPI3
#define MOSI  35                //MISO on SPI3 
#define MISO  37                //MOSI on SPI3
#define SS    38

// #ifdef WebSerial
//   // No incluir WebSerialLite.h
// #else
  #include "WebSerialLite.h"
// #endif

#define DEFAULT_LOG_FILE "/log.txt"
#define LOG_FOLDER_PATH "/logs"

#define SD_Logs true

class Logger {
public:
    enum OutputType { HW_SERIAL, WEBSERIAL };

private:
    String filename = DEFAULT_LOG_FILE;
    bool theresSD = false;

protected:

public:
    OutputType currentOutput;
    
    Logger();
    void setupSD();
    void getSDInfo();
    String getFileName();
    void setFileName(DateTime now);
    void writeSD(const String &message,  DateTime now);
    void init(unsigned long baudRate = 115200);
    void setOutput(OutputType output);
    void print(const String &message);
    void println(const String &message);
    void printValue(const String &key, const String &value);
    void printTime(const String &prefix, int hour, int minute, int day, int month);
};

extern Logger logger; 

#endif // LOGGER_H