#include "Logger.h"

Logger logger; 

void Logger::init(unsigned long baudRate) {
    #ifdef WebSerial_h // Verifica si WebSerialLite.h está incluido 
        // find way to init the webserial from here
    #else
        outputStream.begin(baudRate);
    #endif

    #ifdef SD_Logs
        if (!SD.begin()) {
            outputStream.println("SD Card failed, or not present");
        }
    #endif
}

void Logger::print(const String &message) {
    outputStream.print(message);
}

void Logger::println(const String &message) {
    outputStream.println(message);
}

void Logger::printValue(const String &key, const String &value) {
    outputStream.println(key + ": " + value);
}

void Logger::printTime(const String &prefix, int hour, int minute, int day, int month) {
    String timeStr = prefix + ": " + 
                     String(hour) + "h " + 
                     String(minute) + "min " + 
                     String(day) + "day " + 
                     String(month) + "month";
    outputStream.println(timeStr);
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
