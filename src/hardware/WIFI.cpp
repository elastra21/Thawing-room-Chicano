// #include "WiFiType.h"
#include "WIFI.h"

AsyncWebServer server(80);

static void handle_update_progress_cb(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (!index){
    int cmd = (filename.indexOf("spiffs") > -1) ? U_SPIFFS : U_FLASH;
    // Update.runAsync(true);
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  }

  if (Update.write(data, len) != len) {
    Update.printError(Serial);
  }

  if (final) {
    if (!Update.end(true)){
      Update.printError(Serial);
    }
  }
}

  String WIFI::setLayOutInfo(const char* site, String extra_prop, String value){ 
    String html = site;

    if (extra_prop != "" && value != "") html.replace(extra_prop, value);
    

    html.replace("{{LOCATION}}", String(hostname));
    html.replace("{{VERSION}}", String(VERSION));

    return html;
  };


/* Message callback of WebSerial */
static void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);
}

String WIFI::generateHTMLForJson(JsonVariant json, String path) {
    String html = "";
    if (json.is<JsonObject>()) {
        for (auto kvp : json.as<JsonObject>()) {
            String new_path = path + (path == "" ? "" : "|") + String(kvp.key().c_str());
            html += generateHTMLForJson(kvp.value(), new_path);
        }
    } else if (json.is<JsonArray>()) {
        int index = 0;
        for (JsonVariant v : json.as<JsonArray>()) {
            String new_path = path + String("[") + index + String("]");
            html += generateHTMLForJson(v, new_path);
            index++;
        }
    } else {
        // Generate HTML form elements based on the type of the JSON value
        html += "<label for='" + path + "'>" + path + ": </label>";
        // if (json.is<bool>()) {
        //     html += "<input type='text' name='" + path + "' value='" + String(json.as<bool>()) + "'" + (json.as<bool>() ? " checked" : "") + "><br>";
        // } else {
            html += "<input type='text' name='" + path + "' value='" + String(json.as<String>()) + "'><br>";
        // }
    }
    return html;
}

bool isBoolValue(String value){
  if (value == "true" || value == "false") return true;
  return false;
}

bool valToBool(String value){
  return value == "true";
}

void handleFileUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (!index) {
    Serial.printf("Subiendo archivo: %s\n", filename.c_str());

    SD.remove(CONFIG_FILE);  // Cambiar el nombre del archivo según sea necesario
    File file = SD.open(CONFIG_FILE, FILE_WRITE);
    if (!file) {
      Serial.println("Error al abrir el archivo para escribir");
      return;
    }
    file.close();
  }

  File file = SD.open(CONFIG_FILE, FILE_APPEND);
  if (file) {
    file.write(data, len);
    file.close();
  }

  if (final) {
    Serial.printf("Archivo subido con éxito: %s\n", filename.c_str());
  }
}

bool WIFI::validateJSON(const String& jsonString) {
  const size_t capacity = JSON_OBJECT_SIZE(2) + 30;
  DynamicJsonDocument doc(capacity);
  DeserializationError error = deserializeJson(doc, jsonString);
  return !error; // Retorna true si no hay error, false si hay error
}


void WIFI::updateJsonFromForm(AsyncWebServerRequest *request, JsonVariant json) {
  int params = request->params();
  for (int i = 0; i < params; i++) {
    AsyncWebParameter* p = request->getParam(i);
    String keyPath = p->name();
    std::vector<String> tokens;
    int last = 0, next = 0;
    while ((next = keyPath.indexOf("|", last)) != -1) {
      tokens.push_back(keyPath.substring(last, next));
      last = next + 1;
    }
    tokens.push_back(keyPath.substring(last));

    JsonVariant cur = json;
    for (size_t j = 0; j < tokens.size(); j++) {
      if (j == tokens.size() - 1) {
        // Handle the array index if it exists
        String val = p->value();
        const bool valIsBool = isBoolValue(val);
        
        if (tokens[j][0] == '[') {
          int index = tokens[j].substring(1, tokens[j].length() - 1).toInt();
          cur[index] = valIsBool ? valToBool(val) : val;
        } 
        else {
          if (valIsBool){
          bool bool_value = valToBool(val);
          cur[tokens[j]] = bool_value;
          } else {
            cur[tokens[j]] = val;
          }
            
        }
      } 
      else {
        // Navigate through the JSON
        if (tokens[j][0] == '[') {
          int index = tokens[j].substring(1, tokens[j].length() - 1).toInt();
          cur = cur[index];
        } else {
          cur = cur[tokens[j]];
        }
      }
    }
  }
}


void WIFI::init(const char* ssid, const char* password, const char* hostname){
  strncpy(this->ssid, ssid, sizeof(this->ssid) - 1);
  this->ssid[sizeof(this->ssid) - 1] = '\0';  // Asegurarse de que esté terminado con '\0'

  strncpy(this->password, password, sizeof(this->password) - 1);
  this->password[sizeof(this->password) - 1] = '\0';  // Asegurarse de que esté terminado con '\0'

  strncpy(this->hostname, hostname, sizeof(this->hostname) - 1);
  this->hostname[sizeof(this->hostname) - 1] = '\0';  // Asegurarse de que esté terminado con '\0'
}

void WIFI::setUpWebServer(bool brigeSerial){
  /*use mdns for host name resolution*/
  while (!MDNS.begin(hostname)){ // http://esp32.local
    DEBUG("Error setting up MDNS responder!");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  
  DEBUG("mDNS responder started Pinche Hugo");

  // Función para autenticación básica en todas las rutas
  auto checkAuth = [&](AsyncWebServerRequest *request) {
    if (!request->authenticate(www_username, www_password)) {
      request->requestAuthentication();
      return false;
    }
    return true;
  };


  // ======================== Static Files ========================

  server.on("/style.css", HTTP_GET, [&checkAuth](AsyncWebServerRequest *request){
    if(!checkAuth(request)) return;
    request->send(200, "text/css", STYLE_CSS);
  });

  server.on("/config.txt", HTTP_GET, [&checkAuth](AsyncWebServerRequest *request){
    if(!checkAuth(request)) return;
    if (SD.exists(CONFIG_FILE)) {
        request->send(SD, CONFIG_FILE, "application/json", true);
    } else {
        request->send(404, "text/plain", "Configuration file not found");
    }
  });

  server.on("/defaultParameters.txt", HTTP_GET, [&checkAuth](AsyncWebServerRequest *request){
    if(!checkAuth(request)) return;
    if (SD.exists("/defaultParameters.txt")) {
        request->send(SD, "/defaultParameters.txt", "application/json", true);
    } else {
        request->send(404, "text/plain", "Default Parameters file not found");
    }
  });

// ======================== Routes ========================

    /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, [&](AsyncWebServerRequest *request) {
    if(!checkAuth(request)) return;
    const String doc = setLayOutInfo(SERVER_INDEX_HTML);
    request->send(200, "text/html", doc);
  });

  server.on("/edit-config", HTTP_GET, [&](AsyncWebServerRequest *request) {
    if(!checkAuth(request)) return;
    return request->send(200, "text/html", setLayOutInfo(EDIT_CONFIG_HTML));
  });

  server.on("/edit-settings", HTTP_GET, [&](AsyncWebServerRequest *request) {
    if(!checkAuth(request)) return;
    request->send(200, "text/html", setLayOutInfo(EDIT_SETTINGS_HTML));
  });

  server.on("/logs", HTTP_GET, [&](AsyncWebServerRequest *request){
    if(!checkAuth(request)) return;
    String log_list = "";
    File root = SD.open(LOG_FOLDER_PATH);
    if (!root) {
      request->send(500, "text/plain", "Failed to open logs directory");
      return;
    }
    File file = root.openNextFile();
    while (file) {
      String fileName = file.name();
      log_list += "\""+fileName + "\", ";
      file = root.openNextFile();
    }
    log_list = log_list.substring(0, log_list.length() - 2);  // Remove trailing comma and space
    

    request->send(200, "text/html", setLayOutInfo(LOGS_HTML, "//{{LOGS}}", log_list));
  });

  server.onNotFound([](AsyncWebServerRequest *request) {
      // request->send(SPIFFS, request->url(), String(), false); <------ Buen pishi hack!
      request->send(404, "text/html", "Not found: <u>'"+ request->url() + "'</u>");
  });

  // ======================== SERVER PROCESES ========================

  server.on("/replace-config", HTTP_POST, [&checkAuth](AsyncWebServerRequest *request){
    if(!checkAuth(request)) return;
    request->send(200, "text/plain", "Configuration updated successfully");

  }, handleFileUpload);
  
  /*handling uploading firmware file */
  server.on("/reset", HTTP_POST, [&checkAuth](AsyncWebServerRequest *request) {
    if(!checkAuth(request)) return;
    request->send(200, "text/plain", "Resetting...");
    ESP.restart(); 
  });

  server.on("/update", HTTP_POST, [&checkAuth]( AsyncWebServerRequest *request) {
    if(!checkAuth(request)) return;
    request->send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart(); 
  }, handle_update_progress_cb);

    // Manejar la descarga de archivos
  server.on("/download_log", HTTP_GET, [&checkAuth](AsyncWebServerRequest *request){
    if(!checkAuth(request)) return;
    if (request->hasParam("file")) {
      String fileName = request->getParam("file")->value();
      String full_path = LOG_FOLDER_PATH + String("/") + fileName;
      logger.println(("Downloading file: " + full_path).c_str());
      File file = SD.open(full_path);
      if (file) {
        request->send(file, file.name(), "application/octet-stream");
        file.close();
      } else {
        request->send(404, "text/plain", "File not found");
      }
    } else {
      request->send(400, "text/plain", "File parameter missing");
    }
  });

  server.on("/update-config", HTTP_POST, [&](AsyncWebServerRequest *request) {
  if(!checkAuth(request)) return;

  String body = "";
  if (request->hasParam("body", true)) {
    body = request->getParam("body", true)->value();
  }

  File file = SD.open(CONFIG_FILE, "r");
  if (!file) {
      request->send(500, "text/plain", "Failed to open config file for writing");
      return;
  }
  DynamicJsonDocument doc(4096);  // Ajusta el tamaño según tu archivo JSON

  DeserializationError error = deserializeJson(doc, file);
    if (error) {
        file.close();
        request->send(500, "text/plain", "Failed to parse config file");
        return;
    }

    file.close();  // Close the file to reset the file pointer

    updateJsonFromForm(request, doc);

    // printing doc
    Serial.println("Printing doc");
    serializeJson(doc, Serial);
    Serial.println();

    // Re-open the file for writing
    file = SD.open(CONFIG_FILE, "w");
    if (serializeJson(doc, file) == 0) {
        file.close();
        request->send(500, "text/plain", "Failed to write to file");
    } else {
        file.close();
        request->send(200, "text/plain", "Configuration updated successfully");
    }
  });

  server.on("/toggle-output", HTTP_GET, [&](AsyncWebServerRequest *request) {
    if(!checkAuth(request)) return;
    if (logger.currentOutput == Logger::HW_SERIAL) {
      logger.setOutput(Logger::WEBSERIAL);
    } else {
      logger.setOutput(Logger::HW_SERIAL);
    }
    request->send(200, "text/plain", "Output toggled");
  });

  
  if (brigeSerial) {
    WebSerial.begin(&server);
    WebSerial.onMessage(recvMsg);
  }
  server.begin();
}

String WIFI::getIP(){
  String ip =  MDNS.queryHost("beer-control").toString();
  Serial.println(ip);
  return ip;
}

void WIFI::connectToWiFi(){
  WiFi.begin(ssid, password);
  uint32_t notConnectedCounter = 0;
  EEPROM.begin(32);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    DEBUG("Wifi connecting...");
      
    notConnectedCounter++;
    if(notConnectedCounter > 7) { // Reset board if not connected after 5s
      DEBUG("Resetting due to Wifi not connecting...");
      const uint8_t num_of_tries = EEPROM.readInt(1);
      if (num_of_tries == 3) break;          
      else {
        EEPROM.writeInt(1, num_of_tries + 1);
        EEPROM.commit();
        EEPROM.end();
        ESP.restart();          
      }
    }
  }

  EEPROM.writeInt(1, 0);
  EEPROM.commit();
  EEPROM.end();

  DEBUG(("IP address: " + WiFi.localIP().toString()).c_str());
}

void WIFI::setUpOTA(){
  if(isConnected()){
    ArduinoOTA.setHostname(hostname);
    ArduinoOTA.onStart([]() {
    String type;
    type = ArduinoOTA.getCommand() == U_FLASH ? "sketch" : "filesystem";
      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    logger.println("Start updating " + type);
    }).onEnd([]() {
      logger.println("\nEnd");
    }).onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    }).onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) logger.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) logger.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) logger.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) logger.println("Receive Failed");
      else if (error == OTA_END_ERROR) logger.println("End Failed");
    });
    ArduinoOTA.begin();
  }
}

void WIFI::loopOTA(){
  ArduinoOTA.handle();
}

bool WIFI::refreshWiFiStatus(){
  const bool connection = isConnected();
  if (connection != last_connection_state){
    last_connection_state = connection;
    return true;
  }
  return false;
}

bool WIFI::getConnectionStatus(){
  return last_connection_state;
}

bool WIFI::isConnected(){
  return WiFi.status() == WL_CONNECTED;
}

void WIFI::reconnect(){
  WiFi.begin(ssid, password);
  uint8_t timeout = 0;
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  while ( WiFi.status() != WL_CONNECTED ){
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    log_i(" waiting on wifi connection" );
    timeout++;
    if (timeout == 2) return;
  }
}

void WIFI::DEBUG(const char *message){
  // concat prefix to the message with the classname
  char buffer[100];
  snprintf(buffer, sizeof(buffer), "[WIFI]: %s", message);
  logger.println(buffer);
}\

void WIFI::ERROR(ErrorType error){
  char buffer[100];
  snprintf(buffer, sizeof(buffer), " -> WIFI]: %s", errorMessages[error].c_str());
  logger.printError(buffer);
}



