#include "Controller.h"

RTC_DS3231 rtc;
WiFiUDP ntpUDP;
Adafruit_MLX90640 mlx;
TwoWire rtc_i2c = TwoWire(0);
NTPClient timeClient(ntpUDP);

// OneWire oneWire(ONE_WIRE_BUS);         // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
// DallasTemperature temp_sensor_bus(&oneWire);  // PASS our oneWire reference to Dallas Temperature.

const float voltage_per_step = REFERENCE / ADC__RESOLUTION;
const int16_t range = TEMPERATURE_MAX - TEMPERATURE_MIN;
const double temperature_per_step = range / REFERENCE;

Controller::Controller(/* args */) {
  setUpLogger();
  DEBUG("Controller created");
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
    DEBUG("Logger set up");
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
  pinMode(VALVE_IO, OUTPUT);

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
    DEBUG("RTC I2C not found");
    delay(1000);
  }
  
}

void Controller::setUpRTC() {
  if (!rtc.begin(&rtc_i2c)) {
    DEBUG("Couldn't find RTC");
    while (1);
  }

  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  DateTime now = rtc.now();
  if (true) {
    DEBUG("RTC time seems invalid. Adjusting to NTP time.");
    
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

  if(isTsContactLess()) setUpIRTc();
}

void Controller::setUpIRTc() {
  DEBUG("Inicializando MLX90640...");

  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &rtc_i2c)) {
    DEBUG("¡Error al iniciar el sensor MLX90640!");
    while (1);
  }

  DEBUG("Sensor MLX90640 iniciado correctamente");
  
  mlx.setRefreshRate(MLX90640_4_HZ);
}

float Controller::getIRTemp() {
  float pixelTemps[32 * 24]; // Array temporal para almacenar las temperaturas de todos los píxeles
  float bottomTemps[ARRAY_SIZE]; // Inicializa con valores infinitos

  for (int i = 0; i < ARRAY_SIZE; i++) bottomTemps[i] = INFINITY;

  if (!mlx.getFrame(pixelTemps)) {
    for (int i = 0; i < 32 * 24; i++) checkAndInsertBottomTemps(pixelTemps[i], bottomTemps);

    // Imprimir los 5 valores más bajos
    // DEBUG(("Top" + String(ARRAY_SIZE)+" temperaturas más bajas:").c_str());
    
    // String message = "";
    // for (int i = 0; i < ARRAY_SIZE; i++) message += String(bottomTemps[i])+ " \t";
    // DEBUG(message.c_str());

    // min, max and avg temps
    // DEBUG(("Min: "+String(getMinTemp(bottomTemps))).c_str());
    // DEBUG(("Max: "+String(getMaxTemp(bottomTemps))).c_str());
    // DEBUG(("Avg: "+String(getAvgBottomTemp(bottomTemps))).c_str());
  
    return getAvgBottomTemp(bottomTemps);
  } 
  
  DEBUG("Error al leer el frame del sensor MLX90640");

  return -1;
}

float Controller::getAvgBottomTemp(float *temps){
  float sum = 0;
  for (int i = 0; i < ARRAY_SIZE; i++) sum += temps[i];
  return sum / ARRAY_SIZE;
}

float Controller::getMinTemp(float *temps) {
  float minTemp = temps[0];
  for (int i = 1; i < ARRAY_SIZE; i++) {
    if (temps[i] < minTemp) minTemp = temps[i];
  }
  return minTemp;
}

float Controller::getMaxTemp(float *temps) {
  float maxTemp = temps[0];
  for (int i = 1; i < ARRAY_SIZE; i++) {
    if (temps[i] > maxTemp) maxTemp = temps[i];
  }
  return maxTemp;
}

void Controller::checkAndInsertBottomTemps(float temp, float *temps) {
  if (temp < temps[ARRAY_SIZE - 1]) {
    temps[ARRAY_SIZE - 1] = temp; // Reemplaza el valor más alto con la nueva temperatura
    for (int i = ARRAY_SIZE - 1; i > 0; i--) {
      if (temps[i] < temps[i - 1]) {
        float tmp = temps[i];
        temps[i] = temps[i - 1];
        temps[i - 1] = tmp;
      }
    }
  }
}


bool Controller::isTsContactLess() {
  return ir_ts;
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
        DEBUG("Error al abrir el archivo de configuración para lectura");
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
    DEBUG("Error al abrir el archivo de configuración para escritura");
    return;
  }

  // Serializa el JSON al archivo
  if (serializeJson(doc, configFile) == 0) {
    DEBUG("Error al escribir en el archivo de configuración");
  }

  configFile.close();
}

void Controller::runConfigFile(char* ssid, char* password, char* hostname, char* ip_address, uint16_t* port, char* mqtt_id, char* username, char* mqtt_password, char* prefix_topic) {
  // Iniciar SPIFFS
  if (!SPIFFS.begin(true)) {
    DEBUG("An error has occurred while mounting SPIFFS");
    return;
  }

  // Leer archivo de configuración
  File file = SPIFFS.open("/config.txt");
  if (!file) {
    DEBUG("Failed to open config file");
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
      DEBUG("Failed to parse config file");
    return;
  }

  // Asignar valores y verificar si están presentes en el JSON
  if (doc.containsKey("SSID")) strlcpy(ssid, doc["SSID"], SSID_SIZE);
  if (doc.containsKey("WIFI_PASSWORD")) strlcpy(password, doc["WIFI_PASSWORD"], PASSWORD_SIZE);
  if (doc.containsKey("HOST_NAME")) strlcpy(hostname, doc["HOST_NAME"], HOSTNAME_SIZE);
  if (doc.containsKey("IP_ADDRESS")) strlcpy(ip_address, doc["IP_ADDRESS"], IP_ADDRESS_SIZE);
  if (doc.containsKey("PORT")) *port = doc["PORT"];
  if (doc.containsKey("USERNAME")) strlcpy(username, doc["USERNAME"], HOSTNAME_SIZE);
  // if (doc.containsKey("TOPIC")) strlcpy(prefix_topic, doc["TOPIC"], HOSTNAME_SIZE);
  if (doc.containsKey("MQTT_ID")) strlcpy(mqtt_id, doc["MQTT_ID"], MQTT_ID_SIZE);
  if (doc.containsKey("MQTT_PASSWORD")) strlcpy(mqtt_password, doc["MQTT_PASSWORD"], MQTT_PASSWORD_SIZE);
  // "IR_TS": {
  //   "eneable": true,
  //   "sample_size": 7
  // }
  if(doc.containsKey("IR_TS")){
    ir_ts = doc["IR_TS"]["eneable"];
    ARRAY_SIZE = doc["IR_TS"]["sample_size"];
  }
  // logging all values
  // DEBUG("SSID: " + String(ssid));
  // DEBUG("WIFI_PASSWORD: " + String(password));
  // DEBUG("HOST_NAME: " + String(hostname));
  // DEBUG("IP_ADDRESS: " + String(ip_address));
  // DEBUG("PORT: " + String(*port));
  // DEBUG("USERNAME: " + String(username));
  // DEBUG("TOPIC: " + String(prefix_topic));
  // DEBUG("MQTT_ID: " + String(mqtt_id));
  // DEBUG("MQTT_PASSWORD: " + String(mqtt_password

}

void Controller::setUpDefaultParameters(stage_parameters &stage1_params, stage_parameters &stage2_params, stage_parameters &stage3_params, room_parameters &room, data_tset &N_tset){
  File file = SPIFFS.open("/defaultParameters.txt", "r");
  if (!file) {
    DEBUG("Error al abrir el archivo de parámetros");
    return;
  }

  String jsonText = file.readString();
  file.close();

  // Parsea el JSON
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, jsonText);
  if (error) {
    DEBUG("Error al parsear el JSON");
    return;
  }

  stage1_params.fanOnTime = doc["stage1"]["f1Ontime"];
  stage1_params.fanOffTime = doc["stage1"]["f1Offtime"];
  stage1_params.sprinklerOnTime = doc["stage1"]["s1Ontime"];
  stage1_params.sprinklerOffTime = doc["stage1"]["s1Offtime"];

  stage2_params.fanOnTime = doc["stage2"]["f1Ontime"];
  stage2_params.fanOffTime = doc["stage2"]["f1Offtime"];
  stage2_params.sprinklerOnTime = doc["stage2"]["s1Ontime"];
  stage2_params.sprinklerOffTime = doc["stage2"]["s1Offtime"];

  stage3_params.fanOnTime = doc["stage3"]["f1Ontime"];
  stage3_params.fanOffTime = doc["stage3"]["f1Offtime"];
  stage3_params.sprinklerOnTime = doc["stage3"]["s1Ontime"];
  stage3_params.sprinklerOffTime = doc["stage3"]["s1Offtime"];

  if(stage1_params.sprinklerOffTime < MIN_OFFTIME_STAGE1 ) stage1_params.sprinklerOffTime = MIN_OFFTIME_STAGE1;
  if(stage2_params.sprinklerOffTime < MIN_OFFTIME_STAGE2 ) stage2_params.sprinklerOffTime = MIN_OFFTIME_STAGE2;
  if(stage3_params.sprinklerOffTime < MIN_OFFTIME_STAGE3 ) stage3_params.sprinklerOffTime  = MIN_OFFTIME_STAGE3;
  

  room.A = doc["setPoint"]["A"];
  room.B = doc["setPoint"]["B"];; 

  N_tset.ts = doc["tset"]["tsSet"];
  N_tset.tc = doc["tset"]["tcSet"];

  // // log all data
  // DEBUG("Stage 1 parameters: ");
  // DEBUG("Fan on time: " + String(stage1_params.fanOnTime));
  // DEBUG("Fan off time: " + String(stage1_params.fanOffTime));
  // DEBUG("Sprinkler on time: " + String(stage1_params.sprinklerOnTime));
  // DEBUG("Sprinkler off time: " + String(stage1_params.sprinklerOffTime));

  // DEBUG("Stage 2 parameters: ");
  // DEBUG("Fan on time: " + String(stage2_params.fanOnTime));
  // DEBUG("Fan off time: " + String(stage2_params.fanOffTime));
  // DEBUG("Sprinkler on time: " + String(stage2_params.sprinklerOnTime));
  // DEBUG("Sprinkler off time: " + String(stage2_params.sprinklerOffTime));

  // DEBUG("Stage 3 parameters: ");
  // DEBUG("Fan on time: " + String(stage3_params.fanOnTime));
  // DEBUG("Fan off time: " + String(stage3_params.fanOffTime));
  // DEBUG("Sprinkler on time: " + String(stage3_params.sprinklerOnTime));
  // DEBUG("Sprinkler off time: " + String(stage3_params.sprinklerOffTime));

  // DEBUG("Room parameters: ");
  // DEBUG("A: " + String(room.A));
  // DEBUG("B: " + String(room.B));

  // DEBUG("Tset parameters: ");
  // DEBUG("Ts: " + String(N_tset.ts));
  // DEBUG("Tc: " + String(N_tset.tc));

}

void Controller::WiFiLoop() {
  if (!isWiFiConnected()) {
    reconnectWiFi();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    return;
  }
}

void Controller::DEBUG(const char *message){
  char buffer[100];
  snprintf(buffer, sizeof(buffer), "[Controller]: %s", message);
  logger.println(buffer);
}

void Controller::turnOnFan(bool value, bool CCW) {
  if (value) {
    digitalWrite(FAN_IO, HIGH);
    if(CCW) digitalWrite(FAN_CCW_IO, HIGH);
    else digitalWrite(FAN_CCW_IO, LOW);
  } else {
    digitalWrite(FAN_IO, LOW);
    digitalWrite(FAN_CCW_IO, LOW);
  }
}