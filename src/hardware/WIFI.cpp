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
  if (!MDNS.begin(hostname)){ // http://esp32.local
    DEBUG("Error setting up MDNS responder!");
    while (1){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
  
  DEBUG("mDNS responder started Pinche Hugo");
  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", INDEX_HTML);
  });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/css", STYLE_CSS);
  });

  server.on("/serverIndex", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", SERVER_INDEX_HTML);
  });
  
  /*handling uploading firmware file */
  server.on("/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Resetting...");
    ESP.restart(); 
  });

  server.on("/update", HTTP_POST, []( AsyncWebServerRequest *request) {
    request->send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart(); 
  }, handle_update_progress_cb);

  server.on("/edit-config", HTTP_GET, [&](AsyncWebServerRequest *request) {
    File file = SPIFFS.open("/config.txt", "r");
    if (!file) {
        request->send(500, "text/plain", "Failed to open config file");
        return;
    }

    DynamicJsonDocument doc(4096);  // Ajusta el tamaño según tu archivo JSON
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
        request->send(500, "text/plain", "Failed to parse config file");
        return;
    }

    String html = "<!DOCTYPE html><html><body>"
                  "<h1>Editar Configuración</h1>"
                  "<form action='/update-config' method='POST'>";
    
    html += generateHTMLForJson(doc);
    
    html += "<input type='submit' value='Actualizar'></form>"
            "<br><a href='/download-config'><button type='button'>Descargar JSON</button></a>"
            "</body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/update-config", HTTP_POST, [&](AsyncWebServerRequest *request) {
    File file = SPIFFS.open("/config.txt", "r");
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
    file = SPIFFS.open("/config.txt", "w");
    if (serializeJson(doc, file) == 0) {
        file.close();
        request->send(500, "text/plain", "Failed to write to file");
    } else {
        file.close();
        request->send(200, "text/plain", "Configuration updated successfully");
    }
  });

  server.on("/download-config", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (SPIFFS.exists("/config.txt")) {
        request->send(SPIFFS, "/config.txt", "application/json", true);
    } else {
        request->send(404, "text/plain", "Configuration file not found");
    }
  });
  
  if (brigeSerial) {
    #ifdef WebSerial_h // Verifica si WebSerialLite.h está incluido 
      WebSerial.begin(&server);
      WebSerial.onMessage(recvMsg);
    #endif // WebSerialLite_h
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
}



