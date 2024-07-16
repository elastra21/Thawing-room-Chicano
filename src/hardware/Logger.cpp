#include "Logger.h"

Logger logger; 

Logger::Logger() : currentOutput(HW_SERIAL) {} // Inicializa con SERIAL por defecto

void Logger::init(unsigned long baudRate) {
    if (currentOutput == HW_SERIAL) {
        Serial.begin(baudRate);
    } else {
        // find way to init the webserial from here
    }
}

void Logger::setFileName(DateTime now) {
    filename = "/log_" + String(now.year()) + "_" + String(now.month()) + "_" + String(now.day()) + "_" + String(now.hour()) + "-" + String(now.minute()) + ".txt";
}

void Logger::writeSD(const String &message, DateTime now) {
    if (!theresSD) {
        Serial.println("No SD card found");
        return;
    }

    File file = SD.open(filename, FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }

   // print message with timestamp
    file.print(now.year(), DEC);
    file.print('/');
    file.print(now.month(), DEC);
    file.print('/');
    file.print(now.day(), DEC);
    file.print(" ");
    file.print(now.hour(), DEC);
    file.print(':');
    file.print(now.minute(), DEC);
    file.print(':');
    file.print(now.second(), DEC);
    file.print(" - ");
    file.println(message);

    file.flush();
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
    theresSD = true;
    digitalWrite(SS, HIGH); // Set CS HIGH to deselect the SD card (inverted logic)
}

void Logger::getSDInfo() {
    if (!theresSD) {
        Serial.println("No SD card found");
        return;
    }

    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No CARD");  
        return;
    }

    Serial.print("Card type:");

    if (cardType == CARD_MMC) {
        Serial.println("MMC");
    } 
    else if (cardType == CARD_SD) {
        Serial.println("SDSC");
    } 
    else if (cardType == CARD_SDHC) {
        Serial.println("SDHC");
    } 
    else {
        Serial.println("UNKNOWN");  
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("Card capacity: %lluMB\n", cardSize);
}

