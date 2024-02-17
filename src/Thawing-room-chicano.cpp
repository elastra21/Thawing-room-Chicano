#include "Thawing-room-chicano.h"

// Stage parameters
stage_parameters stage1_params;
stage_parameters stage2_params;  // fan (F1) and sprinklers (S1) STAGE 2 on and off time
stage_parameters stage3_params;

// Stages
Stage stage_1(2, initStage1);
Stage stage_2(2, initStage2);
Stage stage_3(2, initStage3);

// A & B variables
room_parameters room;

// Temperatures measures of Ta, Ts, Tc, Ti & AvgTs
data_s temp_data;

// Ts & Tc target value
data_tset temp_set;

uint8_t fan_1;
uint8_t fan_2;
uint8_t sprinkler_1;

bool sprinkler_1_state = false;
bool mtr_state = false;  // State of the motor that control the Fan F1
bool mtr2_state = false;

// PID parameters
float pid_output, pid_setpoint;           // value of the PID output
double Kp = 0, Ki = 10, Kd = 0;
double Output, pid_input, Setpoint;

bool R_P = true;
bool R_I = true;
bool R_D = true;

double coef_output = 0;  // Output for the infeed (New Analog Output that will be sent to S1
uint8_t coef_pid = 100;
uint8_t Converted_Output = 0;

bool stop_temp1 = false;
bool stop_temp2 = false;

uint8_t chooseTs = 0;

// Parameters of Stage 2
uint8_t stage2_hour = 0;
uint8_t stage2_minute = 0;
uint8_t stage2_day = 0;
uint8_t stage2_month = 0;

bool stage2_rtc_set = 0;
bool stage2_started = 0;
bool stage3_started = 0;

// ########################### Timers ##########################
uint32_t fan_1_timer = 0UL;               // fan F1 timing
uint32_t pid_computing_timer = 0UL;    // PID computing timing
uint32_t fan_1_stg_2_timmer = 0UL;        // F1 stage 2 timing
uint32_t fan_2_stg_2_timmer = 0UL;        // F1 stage 2 timing
uint32_t sprinkler_1_stg_1_timer = 0UL;         // S1 stage 2 timing
uint32_t sprinkler_1_stg_2_timer = 0UL;         // S1 stage 2 timing
uint32_t fan_1_stg_3_timer = 0UL;         // F1 stage 3 timing
uint32_t sprinkler_1_stg_3_timer = 0UL;         // S1 stage 3 timing
uint32_t get_temp_timer = 0UL;         // temperature acquisition
uint32_t ts_avg_timer = 0UL;           // Ts average timing
uint32_t stg_2_pid_timer = 0UL;        // stage 2 PID
uint32_t turn_on_pid_timer = 0UL;      // stage 2 PID ON
uint32_t turn_off_pid_timer = 0UL;     // stage 2 PID OFF
uint32_t address_sending_timer = 0UL;  // Address Sending
uint32_t A_B_timer = 0UL;              // Stage ON/OFF and A&B PUBLISH

// ######################## Temperature ########################
float TA = 0, TA_F = 0;  //Ta
float TS = 0, TS_F = 0;  //Ts
float TC = 0, TC_F = 0;  //Tc
float TI = 0, TI_F = 0;  //Ti optional

// ########################### Buffer ##########################
float avg_ts = 0.0;              // average surface temperature
float buffer_sum = 0;            // variable to store the buffer_sum of the received values
float buffer[BUFFER_SIZE] = {};  // buffer to store the values
uint8_t buffer_len = 0;
uint8_t buffer_index = 0;  // buffer index

SystemState currentState = IDLE;

MqttClient mqtt;
Controller controller;
TaskHandle_t communicationTask;
PID air_in_feed_PID(&pid_input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  // DIRECT or REVERSE

void backgroundTasks(void* pvParameters) {
  for (;;) {
    controller.WiFiLoop();
    
    if(controller.isWiFiConnected()) {
      mqtt.loop();
      controller.loopOTA();
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void setup() {
  controller.init();

  char SSID[SSID_SIZE];
  char PASS[PASSWORD_SIZE];
  char HOST_NAME[HOSTNAME_SIZE];
  char IP_ADDRESS[IP_ADDRESS_SIZE];
  uint16_t PORT;
  char USERNAME[MQTT_USERNAME_SIZE];
  char PREFIX_TOPIC[MQTT_USERNAME_SIZE];

  controller.runConfigFile(SSID, PASS, HOST_NAME, IP_ADDRESS, &PORT, USERNAME, PREFIX_TOPIC);
  controller.setUpDefaultParameters(stage1_params, stage2_params, stage3_params, room, temp_set);

  setStage(IDLE);

  controller.setUpWiFi(SSID, PASS, HOST_NAME);
  controller.connectToWiFi(/* web_server */ false, /* web_serial */ true, /* OTA */ true);
  controller.setUpRTC();

  mqtt.connect(IP_ADDRESS, PORT, USERNAME);
  mqtt.setCallback(callback);
  mqtt.onConnect(onMQTTConnect);


  xTaskCreatePinnedToCore(backgroundTasks, "communicationTask", 10000, NULL, 1, &communicationTask, 0);

  logger.println("===========> Reboted!! <===========");

  //Turn the PID on
  air_in_feed_PID.SetMode(AUTOMATIC);
  air_in_feed_PID.SetSampleTime(3000);
  air_in_feed_PID.SetTunings(Kp, Ki, Kd);

  delay(750);
}

void loop() {
  // if is for testing porpuse comment this "if" and replace DateTime "now" for: DateTime now(__DATE__, __TIME__); 
  DateTime current_date = controller.getDateTime();
  // DateTime current_date(__DATE__, __TIME__); 

  if (!controller.isRTCConnected()) {  
    logger.println("RTC not connected"); 
    while (true) delay(1000);
  }

  updateTemperature();

  if ((TA) > (LOW_TEMP_LIMIT) && (TA) < (HIGH_TEMP_LIMIT)) TA_F = TA;  // if the temperature over the limit it will not be considered
  if ((TS) > (LOW_TEMP_LIMIT) && (TS) < (HIGH_TEMP_LIMIT)) TS_F = TS;
  if ((TC) > (LOW_TEMP_LIMIT) && (TC) < (HIGH_TEMP_LIMIT)) TC_F = TC;
  if ((TI) > (LOW_TEMP_LIMIT) && (TI) < (HIGH_TEMP_LIMIT)) TI_F = TI;

  handleInputs();

  handleStage();
}


bool handleInputs(button_type override) {
  button_type button = NONE;
  bool pressed_btn = false;

  if (override) pressed_btn = button = override;

  if      (controller.readDigitalInput(DLY_S_IO)) button = D_START;
  else if (controller.readDigitalInput(START_IO)) button = START;
  else if (controller.readDigitalInput(STOP_IO))  button = STOP;

  if      (button == D_START) setStage(STAGE1);
  else if (button == START)   setStage(STAGE2);
  else if (button == STOP)    stopRoutine(); 

  return !pressed_btn;
}

void handleStage(){
  if      (currentState == STAGE1)  handleStage1();
  else if (currentState == STAGE2)  handleStage2();
  else if (currentState == STAGE3)  handleStage3();

  else if (currentState == IDLE)    idle();
  else if (currentState == ERROR)   stopRoutine();
}

void handleStage1(){
  logger.println("STAGE #1, current step:" + String(stage_1.getCurrentStep()));

  bool sprinkler_time_to_ON = hasIntervalPassed(sprinkler_1_stg_1_timer,stage1_params.sprinklerOffTime, true);
  bool sprinkler_time_to_OFF = hasIntervalPassed(sprinkler_1_stg_1_timer,stage1_params.sprinklerOnTime, true);
  asyncLoopSprinkler(sprinkler_time_to_ON, sprinkler_time_to_OFF);

  // Init stage 1
  if (stage_1.getCurrentStep() == 0) {
    stage_1.init();
    stage_1.nextStep();
  }
  
  // Step #1
  else if (stage_1.getCurrentStep() == 1){
    if (!mtr_state && !controller.readDigitalInput(FAN_IO)){
      controller.writeDigitalOutput(FAN_IO, HIGH);                                                                                       // Turn ON F1
      logger.println("Stage 1 F1 On");
      mtr_state = true;
      fan_1 = 1;  // When M_F1 = 1 ==> ON

      publishStateChange(m_F1, fan_1, "Stage 1 init M_F1 ON published ");
    }

    else if (hasIntervalPassed(fan_1_timer, stage1_params.fanOffTime , true)) stage_1.nextStep();
     
  }

  // Step #2
  else if (stage_1.getCurrentStep() == 2 ){
    // Turn OFF F1 when the time set in the configuration is over
    if (mtr_state && controller.readDigitalInput(FAN_IO)) {
      controller.writeDigitalOutput(FAN_IO, LOW);
      logger.println("Stage 1 F1 Off");
      mtr_state = false;
      fan_1 = 2;  // When M_F1 = 2 ==> OFF

      publishStateChange(m_F1, fan_1, "Stage 1 init M_F1 OFF published ");
    }

    else if (hasIntervalPassed(fan_1_timer, stage1_params.fanOnTime, true)) stage_1.nextStep();
  }

  // Step #3
  else if (stage_1.getCurrentStep() == 3){
    if (!mtr_state && !controller.readDigitalInput(FAN_IO)){
      controller.writeDigitalOutput(FAN_IO, HIGH);                                                                                       // Turn ON F1
      logger.println("Stage 1 F1 On");
      mtr_state = true;
      fan_1 = 1;  // When M_F1 = 1 ==> ON !!!!!!!!!!!! SHOULD BE CCW

      publishStateChange(m_F1, fan_1, "Stage 1 init M_F1 ON published ");
    }

    else if (hasIntervalPassed(fan_1_timer, stage1_params.fanOffTime , true)) stage_1.nextStep();
  }

  // Step #4
  else if (stage_1.getCurrentStep() == 4){
    if (mtr_state && controller.readDigitalInput(FAN_IO)) {
      controller.writeDigitalOutput(FAN_IO, LOW);
      logger.println("Stage 1 F1 Off");
      mtr_state = false;
      fan_1 = 2;  // When M_F1 = 2 ==> OFF

      publishStateChange(m_F1, fan_1, "Stage 1 init M_F1 OFF published ");
    }

    else if (hasIntervalPassed(fan_1_timer, stage1_params.fanOnTime, true)) stage_1.setStep(1);
  }

  DateTime current_date(__DATE__, __TIME__); 
  
  bool is_after_stage2_time = current_date.hour() >= stage2_hour && current_date.minute() >= stage2_minute;
  bool is_after_stage_2_date = current_date.day() >= stage2_day && current_date.month() >= stage2_month;

  if (is_after_stage2_time && is_after_stage_2_date){
    stage_1.destroy();
    setStage(STAGE2);
  }
  //  Stage 1 Process 
}

void handleStage2(){
  logger.println("STAGE #2, current step:" + String(stage_2.getCurrentStep()));

  // Loop of async process
  bool sprinkler_time_to_ON = hasIntervalPassed(sprinkler_1_stg_2_timer,stage2_params.sprinklerOffTime, true);
  bool sprinkler_time_to_OFF = hasIntervalPassed(sprinkler_1_stg_2_timer,stage2_params.sprinklerOnTime, true);
  asyncLoopSprinkler(sprinkler_time_to_ON, sprinkler_time_to_OFF);

  // Init stage 2
  if (stage_2.getCurrentStep() == 0) {
    stage_2.init(); 
    stage_2.nextStep();
    delay(5000);
  }

  // Step #1
  else if (stage_2.getCurrentStep() == 1){
    if (!mtr_state && hasIntervalPassed(fan_1_stg_2_timmer, stage2_params.fanOffTime, true)) {
      controller.writeDigitalOutput(FAN_IO, HIGH);  // Output of F1
      logger.println("Stage 2 F1 On");
      mtr_state = true;
      fan_1 = 1;  // When M_F1 = 1 ==> ON

      publishStateChange(m_F1, fan_1, "Stage 2 F1 Start published ");
    }

    // Turn OFF F1 when time is over
    if (mtr_state && hasIntervalPassed(fan_1_stg_2_timmer, stage2_params.fanOnTime, true) ){
      controller.writeDigitalOutput(FAN_IO, LOW);
      logger.println("Stage 2 F1 Off");
      mtr_state = false;
      fan_1 = 2;  // When M_F1 = 2 ==> OFF

      publishStateChange(m_F1, fan_1, "Stage 2 F1 Stop published ");
    }

    // Calculate the Setpoint every 3 seconds in Function of Ta with the formula : Setpoint = A*(B-Ta)
    if (hasIntervalPassed(pid_computing_timer, 3000)) {
      Setpoint = (-(room.A * (temp_data.avg_ts)) + room.B);  //use the average of the temperature over the x last minuites
      pid_setpoint = float(Setpoint);

      mqtt.publishData(SETPOINT, pid_setpoint);

      logger.println("Setpoint published");
    }

    // Activate the PID when F1 ON
    if (mtr_state && hasIntervalPassed(turn_on_pid_timer, 3000)) {
      pid_input = TA_F;
      coef_output = (coef_pid * Output) / 100;  // Transform the Output of the PID to the desired max value
      logger.println(String(coef_output));
      air_in_feed_PID.Compute();
      // analogWrite(A0_5, Output);
      controller.writeAnalogOutput(AIR_PWM, Output);
      Converted_Output = ((Output - 0) / (255 - 0)) * (10000 - 0) + 0;
      logger.println("Converted_Output is " + String(Converted_Output));
    }

    // Put the PID at 0 when F1 OFF
    if (!mtr_state && hasIntervalPassed(turn_off_pid_timer, 3000)) {
      //Setpoint = 0;
      pid_input = 0;
      Output = 0;
      coef_output = 0;
      // analogWrite(A0_5, Output);
      controller.writeAnalogOutput(AIR_PWM, Output);
      Converted_Output = ((Output - 0) / (255 - 0)) * (10000 - 0) + 0;
      logger.println("Converted_Output is " + String(Converted_Output));
    }
  }

  // Step #2
  else if (stage_2.getCurrentStep() == 2 ){

  }

  bool isReadyForStage3 = TS_F >= temp_set.ts && TC_F >= temp_set.tc;

  if (isReadyForStage3 ){
    stage_2.destroy();
    setStage(STAGE3);
  } 
  //  Stage 2 Process
}

void handleStage3(){
  logger.println("STAGE #3, current step:" + String(stage_3.getCurrentStep()));

  // Loop of async process
  bool sprinkler_time_to_ON = hasIntervalPassed(sprinkler_1_stg_3_timer,stage3_params.sprinklerOffTime, true);
  bool sprinkler_time_to_OFF = hasIntervalPassed(sprinkler_1_stg_3_timer,stage3_params.sprinklerOnTime, true);
  asyncLoopSprinkler(sprinkler_time_to_ON, sprinkler_time_to_OFF);

  // Init stage 3
  if (stage_3.getCurrentStep() == 0) {
    stage_3.init();
    stage_3.nextStep();
    delay(5000);
  }

  // Step #1
  else if (stage_3.getCurrentStep() == 1){
    stage_3.nextStep();
    delay(5000);
  }

  else if (stage_3.getCurrentStep() == 2){
    stage_3.destroy();
    stopRoutine();
  }
}

void idle(){
  
}

//// fct Callback ==> RECEIVE MQTT MESSAGES ////////////////////////////////////////////////////////////////////
void callback(char *topic, byte *payload, unsigned int len) {
  logger.println("Message arrived [" + String(topic) + "]");

   // STOP
  if (mqtt.isTopicEqual(topic, sub_stop) ) {
    const bool value = mqtt.responseToInt(payload, len);
    if (!value) return;

    handleInputs(STOP);
    logger.println("stop BUTTON PRESSED ON NODE RED" );
  }

  // Choose TS
  if (mqtt.isTopicEqual(topic, sub_chooseTs)) {
    chooseTs = mqtt.responseToInt(payload, len);
    logger.println("Ts is now IR" + String(chooseTs));
  }

  if (currentState != IDLE) return;

      // Delayed start timing
  if (mqtt.isTopicEqual(topic, sub_hours)) {
    stage2_hour = mqtt.responseToFloat(payload, len);
    logger.println("Stage 2 Hours set to: " + String(stage2_minute));
  }

  if (mqtt.isTopicEqual(topic, sub_minutes)) {
    stage2_minute = mqtt.responseToFloat(payload, len);
    logger.println("Stage 2 Minutes set to: " + String(stage2_minute));
  }

  if (mqtt.isTopicEqual(topic, sub_day)) {
    stage2_day = mqtt.responseToFloat(payload, len);
    logger.println("Stage 2 Day set to: " + String(stage2_day));
  }

  if (mqtt.isTopicEqual(topic, sub_month)) {
    stage2_month = mqtt.responseToFloat(payload, len);
    logger.println("Stage 2 Month set to: " + String(stage2_month));
  }

  bool update_default_parameters = false;

  //F1 stg1 on/off time
  if (mqtt.isTopicEqual(topic, sub_f1_st1_ontime)) {
    stage1_params.fanOnTime  = mqtt.responseToFloat(payload, len);
    logger.println("F1 Stage 1 on time set to: " + String(stage1_params.fanOnTime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_f1_st1_offtime)) {
    stage1_params.fanOffTime = mqtt.responseToFloat(payload, len);
    logger.println("F1 Stage 1 off time set to: " + String(stage1_params.fanOffTime) + " MINS");
    update_default_parameters = true;
  }

  // F1 and S1 STAGE 2 on/off time
  if (mqtt.isTopicEqual(topic, sub_f1_st2_ontime)) {
    stage2_params.fanOnTime = mqtt.responseToFloat(payload, len);
    logger.println("F1 Stage 2 on time set to: " + String(stage2_params.fanOnTime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_f1_st2_offtime)) {
    stage2_params.fanOffTime  = mqtt.responseToFloat(payload, len);
    logger.println("F1 Stage 2 off time set to: " + String(stage2_params.fanOffTime ) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_s1_st2_ontime)) {
    stage2_params.sprinklerOnTime = mqtt.responseToFloat(payload, len);
    logger.println("S1 Stage 2 on time set to: " + String(stage2_params.sprinklerOnTime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_s1_st2_offtime)) {
    stage2_params.sprinklerOffTime = mqtt.responseToFloat(payload, len);
    logger.println("S1 Stage 2 off time set to: " + String(stage2_params.sprinklerOffTime) + " MINS");
    update_default_parameters = true;
  }

  // F1 and S1 STAGE 3 on/off time
  if (mqtt.isTopicEqual(topic, sub_f1_st3_ontime)) {
    stage3_params.fanOnTime = mqtt.responseToFloat(payload, len);
    logger.println("F1 Stage 3 on time set to: " + String(stage3_params.fanOnTime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_f1_st3_offtime)) {
    stage3_params.fanOffTime = mqtt.responseToFloat(payload, len);
    logger.println("F1 Stage 3 off time set to: " + String(stage3_params.fanOffTime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_s1_st3_ontime)) {
    stage3_params.sprinklerOnTime = mqtt.responseToFloat(payload, len);
    logger.println("S1 Stage 3 on time set to: " + String(stage3_params.sprinklerOnTime) + " MINS");
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_s1_st3_offtime)) {
    stage3_params.sprinklerOffTime = mqtt.responseToFloat(payload, len);
    logger.println("S1 Stage 3 off time set to: " + String(stage3_params.sprinklerOffTime) + " MINS");
    update_default_parameters = true;
  }

  // Sub A and Sub B value update
  if (mqtt.isTopicEqual(topic, sub_A)) {
    room.A = mqtt.responseToFloat(payload, len);
    // room.A = atoi((char *)payload);
    logger.println("A set to: " + String(room.A));
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_B)) {
    room.B = mqtt.responseToFloat(payload, len);
    logger.println("B set to: " + String(room.B));
    update_default_parameters = true;
  }

  // PID update
  if (mqtt.isTopicEqual(topic, sub_P)) {
    Kp = mqtt.responseToFloat(payload, len);
    logger.println("P set to: " + String(Kp));
    R_P = 1;
  }

  if (mqtt.isTopicEqual(topic, sub_I)) {
    Ki = mqtt.responseToFloat(payload, len);
    logger.println("I set to: " + String(Ki));
    R_I = 1;
  }

  if (mqtt.isTopicEqual(topic, sub_D)) {
    Kd = mqtt.responseToFloat(payload, len);
    logger.println("D set to: " + String(Kd));
    R_D = 1;
  }

  if (R_P == 1 && R_I == 1 && R_D == 1) {
    air_in_feed_PID.SetTunings(Kp, Ki, Kd);
    logger.println("New PID parameter updated");
    R_P = R_I = R_D = 0;
  }

  if (mqtt.isTopicEqual(topic, sub_coefPID)) {
    coef_pid = mqtt.responseToInt(payload, len);
    logger.print("coef PID : " + String(coef_pid));
  }

  // Target temperature Ts & Tc update
  if (mqtt.isTopicEqual(topic, sub_ts_set)) {
    temp_set.ts = mqtt.responseToFloat(payload, len);
    logger.println("Ts Condition set to: " + String(temp_set.ts));
    update_default_parameters = true;
  }

  if (mqtt.isTopicEqual(topic, sub_tc_set)) {
    temp_set.tc = mqtt.responseToFloat(payload, len);
    // Tc_cond = N_tset->N_tc_set;
    logger.println("Tc Condition set to: " + String(temp_set.tc));
    update_default_parameters = true;
  }

  // START  
  if (mqtt.isTopicEqual(topic, sub_start)) {
    const bool value = mqtt.responseToInt(payload, len);
    if (!value) return;

    handleInputs(START);
    logger.println("START BUTTON PRESSED ON NODE RED");
  }
  
  // D_START
  if (mqtt.isTopicEqual(topic, sub_d_start)) {
    const bool value = mqtt.responseToInt(payload, len);
    if (!value) return;
    
    handleInputs(D_START);
    logger.println("d_start BUTTON PRESSED ON NODE RED");
  }
}

void stopRoutine() {
  // if (!stop_temp1) {
    logger.println("PROCESS STOP INITIATED");
    controller.writeDigitalOutput(STAGE_1_IO, LOW);
    controller.writeDigitalOutput(STAGE_2_IO, LOW);
    controller.writeDigitalOutput(STAGE_3_IO, LOW);
    controller.writeDigitalOutput(VALVE_IO, LOW);
    controller.writeDigitalOutput(FAN_IO, LOW);
    controller.writeAnalogOutput(AIR_PWM, 0);

    Output = 0;
    coef_output = 0;
    stop_temp1 = true;

    fan_1 = sprinkler_1 = 2;

    mqtt.publishData(m_F1, fan_1);
    mqtt.publishData(m_S1, sprinkler_1);
    setStage(IDLE);

    stage_1.destroy();
    stage_2.destroy();
    stage_3.destroy();

    logger.println("Stage 0 Status Send packet ");
  // }

  if (!stop_temp2) {
    // mtr_state = sprinkler_1_state = START1 = START2 = stage2_started = stage3_started = stage2_rtc_set = false;
    stop_temp2 = true;
  }

  if (stop_temp2) {
    logger.println("PROCESS STOPPED");
    // stop_temp1 = stop_temp2 = STOP = false;
  }
}

bool isValidTemperature(float temp, float minTemp, float maxTemp, const String& sensorName) {
  bool is_valid = temp < minTemp || temp > maxTemp;
  // if(is_valid) sendTemperaturaAlert(temp, sensorName);

  return is_valid;
}

void updateTemperature() {
  // controller.updateProbesTemperatures();

  float ta_raw = controller.readTempFrom(TA_AI);
  float ts_raw = controller.readTempFrom(TS_AI);
  float tc_raw = controller.readTempFrom(TC_AI);

  TA = isValidTemperature(ta_raw, TA_MIN, TA_MAX, "TA") ? ta_raw : TA_DEF;
  TS = isValidTemperature(ts_raw, TS_MIN, TS_MAX, "TS") ? ts_raw : TS_DEF;
  TC = isValidTemperature(tc_raw, TC_MIN, TC_MAX, "TC") ? tc_raw : TC_DEF;
  // TI = controller.getOneWireTempFrom(controller.ADDRESS_TI);  // Assuming TI doesn't need validation
}

void sendTemperaturaAlert(float temp, String sensor){
  const String msg = "{\"temp\":" + String(temp) + ", \"sensor\":" + sensor + "}";
  mqtt.publishData(SPOILED_SENSOR, msg);
}

void setStage(SystemState Stage) {
  currentState = Stage;
  mqtt.publishData(STAGE, Stage);
}

bool hasIntervalPassed(uint32_t &previousMillis, uint32_t interval, bool to_min) {
  if(to_min) interval *= 60000;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Restablecer el temporizador después de que ha pasado el intervalo
    return true;
  }
  return false; 
}

void publishStateChange(const char* topic, int state, const String& message) {
  mqtt.publishData(topic, state);
  logger.println(message);
}

void aknowledgementRoutine(){
  mqtt.publishData(STAGE, currentState);
  mqtt.publishData(m_F1, fan_1);
  // mqtt.publishData(m_F2, fan_2);
  mqtt.publishData(m_S1, sprinkler_1);
  mqtt.publishData(PID_OUTPUT, coef_output);

    // STAGE 1
  mqtt.publishData(ACK_F1_ST1_ONTIME, stage1_params.fanOnTime);
  mqtt.publishData(ACK_F1_ST1_OFFTIME, stage1_params.fanOffTime);

  // STAGE 2
  mqtt.publishData(ACK_F1_ST2_ONTIME, stage2_params.fanOnTime);
  mqtt.publishData(ACK_F1_ST2_OFFTIME, stage2_params.fanOffTime);
  mqtt.publishData(ACK_S1_ST2_ONTIME, stage2_params.sprinklerOnTime);
  mqtt.publishData(ACK_S1_ST2_OFFTIME, stage2_params.sprinklerOffTime);

  // STAGE 3
  mqtt.publishData(ACK_F1_ST3_ONTIME, stage3_params.fanOnTime);
  mqtt.publishData(ACK_F1_ST3_OFFTIME, stage3_params.fanOffTime);
  mqtt.publishData(ACK_S1_ST3_ONTIME, stage3_params.sprinklerOnTime);
  mqtt.publishData(ACK_S1_ST3_OFFTIME, stage3_params.sprinklerOffTime);

  // A & B
  mqtt.publishData(ACK_A, room.A);
  mqtt.publishData(ACK_B, room.B);
}

void getTsAvg() {
  // if (millis() - ts_avg_timer >= AVG_RESOLUTION)
  if (buffer_len < BUFFER_SIZE) { //if buffer not full, we add the value
      buffer_sum += TS_F;
      buffer[buffer_len] = TS_F;
      buffer_len++;
    }
    else { //buffer full, we remove the oldest value and add the new one
      buffer_sum -= buffer[buffer_index];
      buffer[buffer_index] = TS_F;
      buffer_sum += TS_F;
      buffer_index = (buffer_index + 1) % BUFFER_SIZE; // update the buffer index
    }
    
    avg_ts = buffer_sum/buffer_len;

  mqtt.publishData(AVG_TS_TOPIC, temp_data.avg_ts);
}

void publishPID(){
  if (hasIntervalPassed(stg_2_pid_timer, TIME_ACQ_DELAY + 1)) {
    logger.println("Soft PID Actual Output is" + String(Output));
    const float output_float = float(coef_output);
    pid_output = ((output_float - 0) / (255 - 0)) * (100 - 0) + 0;
    logger.println("PID Output /100 is" + String(pid_output));

    mqtt.publishData(PID_OUTPUT, pid_output);
  }
}

void publishTemperatures(DateTime &current_date) {
  temp_data.ta = TA_F;
  temp_data.ts = TS_F;
  temp_data.tc = TC_F;
  temp_data.ti = TI_F;
  temp_data.avg_ts = avg_ts;

  mqtt.publishData(TA_TOPIC, temp_data.ta);
  mqtt.publishData(TS_TOPIC, temp_data.ts);
  mqtt.publishData(TC_TOPIC, temp_data.tc);
  mqtt.publishData(TI_TOPIC, temp_data.ti);

  // for debug purpose
  logger.println("Average: " + String(temp_data.avg_ts));
  logger.println("Ts: " + String(TS));
  logger.println("TC: " + String(TC));
  logger.println("Ta: " + String(TA));
  // logger.println("Nstart: " + String(remote_start));
  // logger.println("Nstop: " + String(remote_stop));
  logger.println("A variable: " + String(room.A));
  logger.println("B variable: " + String(room.B));
  logger.println("P variable: " + String(Kp));
  logger.println("I variable: " + String(Ki));
  logger.println("D variable: " + String(Kd));
  logger.println("setpoint raw: " + String(Setpoint));
  logger.println("setpoint: " + String(pid_setpoint));

  logger.printTime("Time:", current_date.hour(), current_date.minute(), current_date.day(), current_date.month());
  logger.printTime("Stage 2 Time:", stage2_hour, stage2_minute, stage2_day, stage2_month);
}

void initStage1(){
  logger.println("Stage 1 Initiated");

  fan_1 = 2;
  fan_2 = 0;
  sprinkler_1 = 2;

  mqtt.publishData(m_F1, fan_1);
  logger.println("Stage 1 init M_F1 stop published ");

  mqtt.publishData(m_F2, fan_2);
  logger.println("Stage 1 init M_F2 stop published ");

  mqtt.publishData(m_S1, sprinkler_1);
  logger.println("Stage 1 init M_S1 stop published");

  controller.writeDigitalOutput(STAGE_1_IO, LOW);
  controller.writeDigitalOutput(STAGE_2_IO, LOW);
  controller.writeDigitalOutput(STAGE_3_IO, LOW);
  controller.writeDigitalOutput(VALVE_IO, LOW);
  controller.writeDigitalOutput(FAN_IO, LOW);
  controller.writeDigitalOutput(FAN2_IO, LOW);

  controller.writeDigitalOutput(STAGE_1_IO, HIGH);  // Turn On the LED of Stage 1
  
  fan_1_timer = millis() - (stage1_params.fanOnTime * MINS);
}

void initStage2(){
  logger.println("Stage 2 Initiated");

  fan_1 = 2;
  fan_2 = 0;
  sprinkler_1 = 2;

  controller.writeDigitalOutput(STAGE_1_IO, LOW);
  controller.writeDigitalOutput(STAGE_2_IO, LOW);
  controller.writeDigitalOutput(STAGE_3_IO, LOW);
  controller.writeDigitalOutput(VALVE_IO, LOW);
  controller.writeDigitalOutput(FAN_IO, LOW);
  controller.writeDigitalOutput(FAN2_IO, LOW);

  controller.writeDigitalOutput(STAGE_2_IO, HIGH);  // Turn On the LED of Stage 2
  
  mqtt.publishData(m_F1, fan_1);
  logger.println("Stage 2 init M_F1 stop published ");

  mqtt.publishData(m_F2, fan_2);
  logger.println("Stage 2 init M_F2 stop published ");

  mqtt.publishData(m_S1, sprinkler_1);
  logger.println("Stage 2 init M_S1 stop published"); 


  fan_1_stg_2_timmer = millis() - (stage2_params.fanOffTime * MINS);
}

void onMQTTConnect() {
  mqtt.publishData(m_F1, fan_1);
  mqtt.publishData(m_F2, fan_2);
  mqtt.publishData(m_S1, sprinkler_1);
  mqtt.publishData(STAGE, currentState);
  mqtt.publishData(AVG_TS_TOPIC, avg_ts);
  mqtt.publishData(TA_TOPIC, TA);
  mqtt.publishData(TS_TOPIC, TS);
  mqtt.publishData(TC_TOPIC, TC);
  mqtt.publishData(TI_TOPIC, TI);
  mqtt.publishData(PID_OUTPUT, coef_output);
  mqtt.publishData(SETPOINT, Setpoint);
}

void initStage3(){
  logger.println("Stage 3 Initiated");

  Output = 0;
  coef_output = 0;

  fan_1 = 2;  // When M_F1 = 2 ==> OFF
  sprinkler_1 = 2;  // When M_S1 = 2 ==> OFF
  fan_2 = 0; // // When M_F2 = 0 ==> OFF

  controller.writeAnalogOutput(AIR_PWM, 0);
  controller.writeDigitalOutput(STAGE_1_IO, LOW);
  controller.writeDigitalOutput(STAGE_2_IO, LOW);
  controller.writeDigitalOutput(STAGE_3_IO, LOW);
  controller.writeDigitalOutput(VALVE_IO, LOW);
  controller.writeDigitalOutput(FAN_IO, LOW);
  controller.writeDigitalOutput(FAN2_IO, LOW);

  controller.writeDigitalOutput(STAGE_3_IO, HIGH);  // Turn ON the LED of Stage 3

  publishStateChange(m_F2, fan_2, "Stage 3 M_F2 init published ");
  publishStateChange(m_F1, fan_1, "Stage 3 F1 init published ");
  publishStateChange(m_S1, sprinkler_1, "Stage 3 S1 init published ");

  fan_1_stg_3_timer = millis() - (stage3_params.fanOffTime * MINS);
}

void asyncLoopSprinkler(bool sprinkler_time_to_ON, bool sprinkler_time_to_OFF){
  // Should ON the Sprinkler
  if (mtr_state && !sprinkler_1_state && sprinkler_time_to_ON) {    
    controller.writeDigitalOutput(VALVE_IO, HIGH);  // Output of S1
    sprinkler_1_state = true;
    logger.println("Stage 2 S1 ON");
    sprinkler_1 = 1;  // When M_S1 = 1 ==> ON

    publishStateChange(m_S1, sprinkler_1, "Stage 2 S1 Start published ");
  }

  // Should OFF the Sprinkler
  if ((sprinkler_1_state || !mtr_state) && sprinkler_time_to_OFF) {    
    controller.writeDigitalOutput(VALVE_IO, LOW);  // Output of S1
    sprinkler_1_state = false;
    logger.println("Stage 2 S1 OFF");
    sprinkler_1 = 2;  // When M_S1 = 2 ==> OFF

    publishStateChange(m_S1, sprinkler_1, "Stage 2 S1 Stop published ");
  }
}