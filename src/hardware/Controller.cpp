#include "Controller.h"

RTC_DS3231 rtc;
WiFiUDP ntpUDP;
TwoWire rtc_i2c = TwoWire(0);
NTPClient timeClient(ntpUDP);
// OneWire oneWire(ONE_WIRE_BUS);         // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
// DallasTemperature temp_sensor_bus(&oneWire);  // PASS our oneWire reference to Dallas Temperature.

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

  for (uint8_t i = 0; i < inputs_size; i++) pinMode(inputs[i], INPUT_PULLUP);
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

void Controller::updateDefaultParameters(stage_parameters &stage1_params, stage_parameters &stage2_params, stage_parameters &stage3_params, room_parameters &room, data_tset &N_tset ){
  // Abre el archivo de configuración existente
  File configFile = SPIFFS.open("/defaultParameters.txt", FILE_READ);
  if (!configFile) {
    Serial.println("Error al abrir el archivo de configuración para lectura");
    return;
  }

  // Lee el contenido en una cadena
  String content = configFile.readString();
  configFile.close();

  // Parsea el objeto JSON del archivo
  StaticJsonDocument<1024> doc; // Cambiado a StaticJsonDocument
  auto error = deserializeJson(doc, content);
  if (error) {
    Serial.println("Error al parsear el archivo de configuración");
    return;
  }

  // Update the values
  doc["stage1"]["f1Ontime"] = stage1_params.fanOnTime;
  doc["stage1"]["f1Offtime"] = stage1_params.fanOffTime;

  doc["stage2"]["f1Ontime"] = stage2_params.fanOnTime;
  doc["stage2"]["f1Offtime"] = stage2_params.fanOffTime;
  doc["stage2"]["s1Ontime"] = stage2_params.sprinklerOnTime;
  doc["stage2"]["s1Offtime"] = stage2_params.sprinklerOffTime;

  doc["stage3"]["f1Ontime"] = stage3_params.fanOnTime;
  doc["stage3"]["f1Offtime"] = stage3_params.fanOffTime;
  doc["stage3"]["s1Ontime"] = stage3_params.sprinklerOnTime;
  doc["stage3"]["s1Offtime"] = stage3_params.sprinklerOffTime;

  doc["setPoint"]["A"] = room.A;
  doc["setPoint"]["B"] = room.B;
  
  doc["tset"]["tsSet"] = N_tset.ts;
  doc["tset"]["tcSet"] = N_tset.tc;

  // Open file for writing
  configFile = SPIFFS.open("/defaultParameters.txt", FILE_WRITE);
  if (!configFile) {
    Serial.println("Error al abrir el archivo de configuración para escritura");
    return;
  }

  // Serializa el JSON al archivo
  if (serializeJson(doc, configFile) == 0) {
    Serial.println("Error al escribir en el archivo de configuración");
  }

  configFile.close();
}

void Controller::runConfigFile(char* ssid, char* password, char* hostname, char* ip_address, uint16_t* port, char* username, char* prefix_topic) {
  // Iniciar SPIFFS
  if (!SPIFFS.begin(true)) {
    logger.println("An error has occurred while mounting SPIFFS");
    return;
  }

  // Leer archivo de configuración
  File file = SPIFFS.open("/config.txt");
  if (!file) {
    logger.println("Failed to open config file");
    return;
  }

  // Tamaño para el documento JSON
  size_t size = file.size();
  std::unique_ptr<char[]> buf(new char[size]);
  file.readBytes(buf.get(), size);
  file.close();

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, buf.get());
  if (error) {
    Serial.println("Failed to parse config file");
    return;
  }

  // Asignar valores y verificar si están presentes en el JSON
  if (doc.containsKey("SSID")) strlcpy(ssid, doc["SSID"], SSID_SIZE);
  if (doc.containsKey("WIFI_PASSWORD")) strlcpy(password, doc["WIFI_PASSWORD"], PASSWORD_SIZE);
  if (doc.containsKey("HOST_NAME")) strlcpy(hostname, doc["HOST_NAME"], HOSTNAME_SIZE);
  if (doc.containsKey("IP_ADDRESS")) strlcpy(ip_address, doc["IP_ADDRESS"], IP_ADDRESS_SIZE);
  if (doc.containsKey("PORT")) *port = doc["PORT"];
  if (doc.containsKey("USERNAME")) strlcpy(username, doc["USERNAME"], HOSTNAME_SIZE);
  if (doc.containsKey("TOPIC")) strlcpy(prefix_topic, doc["TOPIC"], HOSTNAME_SIZE);

  // logging all values
  logger.println("SSID: " + String(ssid));
  logger.println("WIFI_PASSWORD: " + String(password));
  logger.println("HOST_NAME: " + String(hostname));
  logger.println("IP_ADDRESS: " + String(ip_address));
  logger.println("PORT: " + String(*port));
  logger.println("USERNAME: " + String(username));
  logger.println("TOPIC: " + String(prefix_topic));

}

void Controller::setUpDefaultParameters(stage_parameters &stage1_params, stage_parameters &stage2_params, stage_parameters &stage3_params, room_parameters &room, data_tset &N_tset){
  File file = SPIFFS.open("/defaultParameters.txt", "r");
  if (!file) {
    Serial.println("Error al abrir el archivo de parámetros");
    return;
  }

  String jsonText = file.readString();
  file.close();

  // Parsea el JSON
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, jsonText);
  if (error) {
    Serial.println("Error al parsear el JSON");
    return;
  }

  stage1_params.fanOnTime = doc["stage1"]["f1Ontime"];
  stage1_params.fanOffTime = doc["stage1"]["f1Offtime"];

  stage2_params.fanOnTime = doc["stage2"]["f1Ontime"];
  stage2_params.fanOffTime = doc["stage2"]["f1Offtime"];
  stage2_params.sprinklerOnTime = doc["stage2"]["s1Ontime"];
  stage2_params.sprinklerOffTime = doc["stage2"]["s1Offtime"];

  stage3_params.fanOnTime = doc["stage3"]["f1Ontime"];
  stage3_params.fanOffTime = doc["stage3"]["f1Offtime"];
  stage3_params.sprinklerOnTime = doc["stage3"]["s1Ontime"];
  stage3_params.sprinklerOffTime = doc["stage3"]["s1Offtime"];

  room.A = doc["setPoint"]["A"];
  room.B = doc["setPoint"]["B"];; 

  N_tset.ts = doc["tset"]["tsSet"];
  N_tset.tc = doc["tset"]["tcSet"];
}

void Controller::WiFiLoop() {
  if (!isWiFiConnected()) {
    reconnectWiFi();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    return;
  }
}