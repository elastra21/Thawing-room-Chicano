#include "MqttClient.h"

WiFiClient esp32Client;
PubSubClient mqttClient(esp32Client);

// Global reference for internal callback dispatch
MqttClient* g_mqttClientInstance = nullptr;

// Internal callback wrapper that dispatches to registered events
void internalMqttCallback(char* topic, byte* payload, unsigned int length) {
  if (g_mqttClientInstance) {
    g_mqttClientInstance->dispatchEvent(topic, payload, length);
  }
}

MqttClient::MqttClient() {
  g_mqttClientInstance = this;
}

void MqttClient::connect(const char *domain, uint16_t port, const char *id, const char *username, const char *password) {
  strncpy(mqtt_domain, domain, sizeof(mqtt_domain) - 1);
  mqtt_domain[sizeof(mqtt_domain) - 1] = '\0';  // Asegurarse de que esté terminado con '\0'

  mqtt_port = port;

  strncpy(mqtt_id, id, sizeof(mqtt_id) - 1);
  mqtt_id[sizeof(mqtt_id) - 1] = '\0';  // Asegurarse de que esté terminado con '\0'

  strncpy(mqtt_username, username, sizeof(mqtt_username) - 1);
  mqtt_username[sizeof(mqtt_username) - 1] = '\0';  // Asegurarse de que esté terminado con '\0'

  strncpy(mqtt_password, password, sizeof(mqtt_password) - 1);
  mqtt_password[sizeof(mqtt_password) - 1] = '\0';  // Asegurarse de que esté terminado con '\0'

  mqttClient.setServer(domain, port);
  if (mqttClient.connect(mqtt_id, mqtt_username, mqtt_password)) {
    DEBUG("Connection has been established, well done");
    if(callback_connect != NULL) callback_connect();
    subscribeRoutine();
    no_service_available = false;
  } else {
    DEBUG("Looks like the server connection failed...");
  }
}

bool MqttClient::isServiceAvailable() {
  return !no_service_available;
}

void MqttClient::reconnect() {
  static unsigned long lastReconnectAttempt = 0;
  unsigned long now = millis();
  static int reconnectAttempts = 0;

  if (!mqttClient.connected()) {
    if (now - lastReconnectAttempt > 120000 || reconnectAttempts == 0) { // 120000ms = 2 minutos
      lastReconnectAttempt = now;

      if (reconnectAttempts < 5) {
        mqttClient.flush();
        mqttClient.disconnect();
        mqttClient.setServer(mqtt_domain, mqtt_port);
        
        DEBUG("Attempting MQTT connection...");

        if (mqttClient.connect(mqtt_id, mqtt_username, mqtt_password)) {
          DEBUG("connected");
          subscribeRoutine();
          reconnectAttempts = 0; // Resetear los intentos si la conexión es exitosa
        } else {
          DEBUG(("failed, rc= " + String(mqttClient.state()) + ", try again in 2 minutes").c_str());
          reconnectAttempts++;
        }
      } else {
        DEBUG("Max reconnect attempts reached, try again in 2 minutes");
        reconnectAttempts = 0; // Resetear los intentos después de alcanzar el máximo
      }
    }
  } else {
    reconnectAttempts = 0; // Resetear los intentos si ya está conectado
  }
}

void MqttClient::onConnect(std::function<void()> callback) {
  callback_connect = callback;
}

bool MqttClient::isTopicEqual(const char* a, const char* b){
  return strcmp(a, b) == 0;
}

bool MqttClient::isConnected() {
  return mqttClient.connected();
}

void MqttClient::loop() {
  if (!isServiceAvailable()) return;
  if (!mqttClient.connected()) reconnect();
  
  delay(100);
  // vTaskDelay(100 / portTICK_PERIOD_MS);
  mqttClient.loop();
}

void MqttClient::setCallback(std::function<void(char *, uint8_t *, unsigned int)> callback) {
  mqttClient.setCallback(callback);
}

void MqttClient::setEventDispatcher() {
  mqttClient.setCallback(internalMqttCallback);
}

void MqttClient::subscribeRoutine() {
  if (mqttClient.connect(mqtt_username)) {
    DEBUG("connected, subscribing");
    
    for (int i = 0; i < SUB_ARRAY_SIZE; i++) mqttClient.subscribe(topics[i]);

    DEBUG("subscribing done");
  } else DEBUG("not connected, subscribing aborted");
}

void MqttClient::publishData(String topic, double value) {
  if (WiFi.status() != WL_CONNECTED) return;
  char value_buffer[8];
  dtostrf(value, 1, 2, value_buffer);
  mqttClient.publish(topic.c_str(), value_buffer);
}

void MqttClient::publishData(String topic, String value) {
  if (WiFi.status() != WL_CONNECTED) return;
  mqttClient.publish(topic.c_str(), value.c_str());
}

bool MqttClient::refreshMQTTStatus() {
  const bool connection = isConnected();
  if (connection != last_connection_state) {
    last_connection_state = connection;
    return true;
  }
  return false;
}

bool MqttClient::getConnectionStatus() {
  return last_connection_state;
}

float MqttClient::responseToFloat(byte *value, size_t len) {
  String string_builder;
  for (int i = 0; i < len; i++) string_builder += (char)value[i];
  return string_builder.toFloat();
}

int MqttClient::responseToInt(byte *value, size_t len) {
  String string_builder;
  for (int i = 0; i < len; i++) string_builder += (char)value[i];
  return string_builder.toInt();
}

void MqttClient::DEBUG(const char *message){
  // concat prefix to the message with the classname
  char buffer[100];
  snprintf(buffer, sizeof(buffer), "[MqttClient]: %s", message);
  logger.println(buffer);
}

void MqttClient::ERROR(ErrorType error){
  char buffer[100];
  snprintf(buffer, sizeof(buffer), "[ERROR -> MqttClient]: %s", errorMessages[error]);
  logger.println(buffer);
}

void MqttClient::createMqttEvent(const char* topic, std::function<void (char *, uint8_t *, unsigned int)> callback) {
  mqtt_event event;
  event.topic = topic;
  event.callback = callback;
  events.push_back(event);
}

mqtt_event MqttClient::searchEventByTopic(const char* topic) {
  mqtt_event emptyEvent = {nullptr, nullptr};
  
  for (const auto& event : events) {
    if (event.topic && strcmp(event.topic, topic) == 0) {
      return event;
    }
  }
  
  return emptyEvent;
}

void MqttClient::dispatchEvent(char* topic, uint8_t* payload, unsigned int length) {
  mqtt_event event = searchEventByTopic(topic);
  if (event.callback) {
    event.callback(topic, payload, length);
  } else if (fallback_callback) {
    // If no specific event handler found, use fallback callback
    fallback_callback(topic, payload, length);
  }
}

void MqttClient::enableHybridMode(std::function<void (char *, uint8_t *, unsigned int)> fallbackCallback) {
  fallback_callback = fallbackCallback;
  mqttClient.setCallback(internalMqttCallback);
}