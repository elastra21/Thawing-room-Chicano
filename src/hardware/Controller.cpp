#include "Controller.h"

RTC_DS3231 rtc;
WiFiUDP ntpUDP;
TwoWire rtc_i2c = TwoWire(0);
NTPClient timeClient(ntpUDP);

const float voltage_per_step = REFERENCE / ADC__RESOLUTION;
const int16_t range = TEMPERATURE_MAX - TEMPERATURE_MIN;
const double temperature_per_step = range / REFERENCE;

Controller::Controller(/* args */) {
  setUpLogger();
  logger.println("Controller created");
}

Controller::~Controller() {

}

void Controller::init() {
  setUpI2C();
  setUpIOS();
}

void Controller::setUpLogger() {
  #ifdef WebSerial
    logger.init(115200);
    logger.println("Logger set up");
  #else
  
  #endif
}

void Controller::setUpIOS() {
  setUpAnalogInputs();
  setUpAnalogOutputs();
  setUpDigitalInputs();
  setUpDigitalOutputs();

  // Setting to LOW all pulled up pins
  for (uint8_t i = 0; i < pulled_up_size; i++) {
    pinMode(pulled_up[i], OUTPUT);
    digitalWrite(pulled_up[i], LOW);
  }
}


void Controller::setUpAnalogOutputs() {
  ledcSetup(AIR_PWM, FREQ, RESOLUTION);
  ledcAttachPin(AIR_PIN, AIR_PWM);
}

void Controller::setUpDigitalOutputs() {
  for (uint8_t i = 0; i < outputs_size; i++) pinMode(outputs[i], OUTPUT);
}

void Controller::setUpDigitalInputs() {
  //Testing pourpose
  // pinMode(PORT_B0, INPUT_PULLUP);

  for (uint8_t i = 0; i < inputs_size; i++) pinMode(inputs[i], INPUT);
}

void Controller::setUpAnalogInputs() {
  
}

void Controller::setUpI2C() {
  while (!rtc_i2c.begin(I2C_SDA, I2C_SCL)){
    logger.println("RTC I2C not found");
    delay(1000);
  }
  
}

void Controller::setUpRTC() {
  if (!rtc.begin(&rtc_i2c)) {
    logger.println("Couldn't find RTC");
    while (1);
  }
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  DateTime now = rtc.now();
  if (true) {
    logger.println("RTC time seems invalid. Adjusting to NTP time.");
    
    timeClient.begin();
    timeClient.setTimeOffset(SECS_IN_HR * TIME_ZONE_OFFSET_HRS);
    timeClient.setUpdateInterval(SECS_IN_HR);
    timeClient.update();

    delay(1000); 

    long epochTime = timeClient.getEpochTime();

    // Convert received time from Epoch format to DateTime format
    DateTime ntpTime(epochTime);

    // Adjust RTC
    rtc.adjust(ntpTime);
  }
}

bool Controller::isRTCConnected() {
  return rtc.begin(&rtc_i2c);
}

DateTime Controller::getDateTime() {
  return rtc.now();
}

uint64_t Controller::readAnalogInput(uint8_t input) {
  return analogRead(input);
}

bool Controller::readDigitalInput(uint8_t input) {
  return digitalRead(input);
}

void Controller::writeAnalogOutput(uint8_t output, uint8_t value) {
  ledcWrite(AIR_PWM, value);
}

void Controller::writeDigitalOutput(uint8_t output, uint8_t value) {
  digitalWrite(output, value);
}

float Controller::readTempFrom(uint8_t channel) {
  const uint16_t raw_voltage_ch = analogRead(channel); 
  // const float voltage_ch = (raw_voltage_ch * voltage_per_step);
  // Serial.println(voltage_ch);
  // const float temp = (voltage_ch * temperature_per_step) + TEMPERATURE_MIN;
  const float temp = raw_voltage_ch*0.0247 - 52.933; // ramp calculated with excel trhough manual calibration
  return temp;
}

// WIFI CLASS

void Controller::connectToWiFi(bool web_server, bool web_serial, bool OTA) {
  wifi.connectToWiFi();
  if(OTA) wifi.setUpOTA();
  if(web_server) wifi.setUpWebServer(web_serial);
}

void Controller::reconnectWiFi() {
  wifi.reconnect();
}

bool Controller::isWiFiConnected() {
  return wifi.isConnected();
}

bool Controller::refreshWiFiStatus() {
  return wifi.refreshWiFiStatus();
}

bool Controller::getConnectionStatus() {
  return wifi.getConnectionStatus();
}

void Controller::loopOTA() {
  wifi.loopOTA();
}

void Controller::setUpWiFi(const char* ssid, const char* password, const char* hostname) {
  wifi.init(ssid, password, hostname);
}