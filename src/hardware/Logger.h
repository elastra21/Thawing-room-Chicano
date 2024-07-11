#ifndef LOGGER_H
#define LOGGER_H

#include "SD.h"
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

class Logger {
public:
    enum OutputType { HW_SERIAL, WEBSERIAL };

private:

    void setupSD();

public:
    OutputType currentOutput;
    
    Logger();

    void init(unsigned long baudRate = 115200);

    void setOutput(OutputType output);

    void print(const String &message);

    void println(const String &message);

    void printValue(const String &key, const String &value);

    void printTime(const String &prefix, int hour, int minute, int day, int month);
};

extern Logger logger; 

#endif // LOGGER_H