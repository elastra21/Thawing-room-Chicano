#include "Logger.h"

Logger logger; 

Logger::Logger() : currentOutput(HW_SERIAL) {} // Inicializa con SERIAL por defecto

void Logger::init(unsigned long baudRate) {
    if (currentOutput == HW_SERIAL) {
        Serial.begin(baudRate);
    } else {
        // find way to init the webserial from here
    }

    #ifdef SD_Logs
        if (!SD.begin()) {
            Serial.println("SD Card failed, or not present");
        }
    #endif
}

void Logger::setOutput(OutputType output) {
    currentOutput = output;
}

void Logger::print(const String &message) {
    if (currentOutput == HW_SERIAL) {
        Serial.print(message);
    } else {
        WebSerial.print(message);
    }
}

void Logger::println(const String &message) {
    if (currentOutput == HW_SERIAL) {
        Serial.println(message);
    } else {
        WebSerial.println(message);
    }
}

void Logger::printValue(const String &key, const String &value) {
    if (currentOutput == HW_SERIAL) {
        Serial.println(key + ": " + value);
    } else {
        WebSerial.println(key + ": " + value);
    }
}

void Logger::printTime(const String &prefix, int hour, int minute, int day, int month) {
    String timeStr = prefix + ": " + 
                     String(hour) + "h " + 
                     String(minute) + "min " + 
                     String(day) + "day " + 
                     String(month) + "month";
    if (currentOutput == HW_SERIAL) {
        Serial.println(timeStr);
    } else {
        WebSerial.println(timeStr);
    }
}

void Logger::setupSD() {
    SPI.begin(SCK, MISO, MOSI, SS);
    pinMode(SS, OUTPUT);
    digitalWrite(SS, HIGH); // Set CS to HIGH initially (inverted logic)

    // Manually control CS for the SD card initialization
    digitalWrite(SS, LOW); // Set CS LOW to select the SD card (inverted logic)
    if (!SD.begin(SS)) {
        Serial.println("Failed to mount card!");
        digitalWrite(SS, HIGH); // Set CS HIGH to deselect the SD card (inverted logic)
        return;
    }
    digitalWrite(SS, HIGH); // Set CS HIGH to deselect the SD card (inverted logic)
}
