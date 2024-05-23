#ifndef MY_MQTT_H
#define MY_MQTT_H
#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#define MQTT_USERNAME_SIZE 32
#define MQTT_ID_SIZE 32
#define MQTT_PASSWORD_SIZE 32
#define PREFIX_SIZE 32

//             subscribe topics    -------------------------------------------------------------------->
#define sub_hours           "mfp2/hours"
#define sub_minutes         "mfp2/minutes"
#define sub_day             "mfp2/day"
#define sub_month           "mfp2/month"
#define sub_f1_st1_ontime   "mfp2/f1_st1_ontime"
#define sub_f1_st1_offtime  "mfp2/f1_st1_offtime"
#define sub_f1_st2_ontime   "mfp2/f1_st2_ontime"
#define sub_f1_st2_offtime  "mfp2/f1_st2_offtime"
#define sub_s1_st2_ontime   "mfp2/s1_st2_ontime"
#define sub_s1_st2_offtime  "mfp2/s1_st2_offtime"
#define sub_f1_st3_ontime   "mfp2/f1_st3_ontime"
#define sub_f1_st3_offtime  "mfp2/f1_st3_offtime"
#define sub_s1_st3_ontime   "mfp2/s1_st3_ontime"
#define sub_s1_st3_offtime  "mfp2/s1_st3_offtime"
#define sub_A               "mfp2/A"
#define sub_B               "mfp2/B"
#define sub_ts_set          "mfp2/ts_set"
#define sub_tc_set          "mfp2/tc_set"
#define sub_start           "mfp2/start"
#define sub_d_start         "mfp2/d_start"
#define sub_stop            "mfp2/stop"
#define sub_TempAcqDelay    "mfp2/TempAcqDelay"
#define sub_P               "mfp2/P"
#define sub_I               "mfp2/I"
#define sub_D               "mfp2/D"
#define sub_avgTiming       "mfp2/TsAvgTime"      // in ms the sampling rate for Ts calculation
#define sub_tsAvgSpan       "mfp2/TsAvgFifoSpan"  // in minutes the span of the fifo for Ts calculation
#define sub_chooseTs        "mfp2/chooseTs"
#define sub_coefPID         "mfp2/coefPID"

//------------ publish index    -------------------------------------------------------------------->
#define m_F1                "mfp2/M_F1"
#define m_F2                "mfp2/M_F2"
#define m_S1                "mfp2/M_S1"
#define STAGE               "mfp2/stage"
#define AVG_TS_TOPIC        "mfp2/AvgTs"
#define AVG_TC_TOPIC        "mfp2/AvgTc"
#define AVG_TA_TOPIC        "mfp2/AvgTa"
#define TA_TOPIC            "mfp2/Ta"
#define TS_TOPIC            "mfp2/Ts"
#define TC_TOPIC            "mfp2/Tc"
#define TI_TOPIC            "mfp2/Ti"
#define PID_OUTPUT          "mfp2/PID_output"
#define SETPOINT            "mfp2/setpoint"
#define ACK_F1_ST1_ONTIME   "mfp2/ack_f1_st1_ontime"
#define ACK_F1_ST1_OFFTIME  "mfp2/ack_f1_st1_offtime"
#define ACK_F1_ST2_ONTIME   "mfp2/ack_f1_st2_ontime"
#define ACK_F1_ST2_OFFTIME  "mfp2/ack_f1_st2_offtime"
#define ACK_S1_ST2_ONTIME   "mfp2/ack_s1_st2_ontime"
#define ACK_S1_ST2_OFFTIME  "mfp2/ack_s1_st2_offtime"
#define ACK_F1_ST3_ONTIME   "mfp2/ack_f1_st3_ontime"
#define ACK_F1_ST3_OFFTIME  "mfp2/ack_f1_st3_offtime"
#define ACK_S1_ST3_ONTIME   "mfp2/ack_s1_st3_ontime"
#define ACK_S1_ST3_OFFTIME  "mfp2/ack_s1_st3_offtime"
#define ACK_A               "mfp2/ack_A"
#define ACK_B               "mfp2/ack_B"
#define ACK_TS              "mfp2/ack_Ts"
#define ACK_TC              "mfp2/ack_Tc"



class MqttClient {
  public:
    void loop();
    void connect(const char *domain, uint16_t port, const char *id, const char *username, const char *password);
    void reconnect();
    bool isConnected();
    void subscribeRoutine();
    bool refreshMQTTStatus();
    bool isServiceAvailable();
    bool getConnectionStatus();
    void publishData(String topic, double value);
    void publishData(String topic, String value);
    void setCallback(std::function<void (char *, uint8_t *, unsigned int)> callback);
    
    void DEBUG(const char *message);
  private:
    char mqtt_id[MQTT_USERNAME_SIZE];  
    char mqtt_username[MQTT_USERNAME_SIZE];
    char mqtt_password[MQTT_USERNAME_SIZE];
    bool no_service_available = true;
    bool last_connection_state = false;

};
#endif
