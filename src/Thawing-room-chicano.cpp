#include "Thawing-room-chicano.h"

// Stage parameters
stage_parameters stage1_params;
stage_parameters stage2_params;  // fan (F1) and sprinklers (S1) STAGE 2 on and off time
stage_parameters stage3_params;

// Stages
Stage stage_1(2, initStage1, destroyStage1);
Stage stage_2(2, initStage2, destroyStage2);
Stage stage_3(2, initStage3, destroyStage3);

Button start_btn(START_IO); // Connect your button between pin 2 and GND
Button stop_btn(STOP_IO); // Connect your button between pin 3 and GND
Button d_start_button(DLY_S_IO); // Connect your button between pin 4 and GND

// A & B variables
room_parameters room;

// Temperatures measures of Ta, Ts, Tc, Ti & AvgTs
data_s temp_data;

// Ts & Tc target value
data_tset temp_set;

// PID parameters
float pid_output, pid_setpoint;           // value of the PID output
double Kp = 0, Ki = 10, Kd = 0;
double Output, pid_input, Setpoint;

bool kp_has_changed = true;
bool ki_has_changed = true;
bool kd_has_changed = true;

double coef_output = 0;  // Output for the infeed (New Analog Output that will be sent to S1
uint8_t coef_pid = 100;
uint8_t Converted_Output = 0;

bool is_rts_ir = 0;

// Parameters of Stage 2
uint8_t stage2_hour = 0;
uint8_t stage2_minute = 0;
uint8_t stage2_day = 0;
uint8_t stage2_month = 0;

// ########################### Timers ##########################
uint32_t fan_1_timer = 0UL;               // fan F1 timing
uint32_t pid_computing_timer = 0UL;    // PID computing timing
uint32_t fan_1_stg_2_timmer = 0UL;        // F1 stage 2 timing
uint32_t fan_2_stg_2_timmer = 0UL;        // F1 stage 2 timing
uint32_t fan_1_stg_3_timer = 0UL;         // F1 stage 3 timing
uint32_t sprinkler_1_stg_1_timer = 0UL;         // S1 stage 2 timing
uint32_t sprinkler_1_stg_2_timer = 0UL;         // S1 stage 2 timing
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
SensorBuffer sensorTs(BUFFER_SIZE);  // Crear una instancia para el sensor Ts
SensorBuffer sensorTc(10);  // Crear otra instancia para el sensor Tc
SensorBuffer sensorTa(10);  // Crear otra instancia para el sensor Ta

// SystemState currentState = IDLE;
StageState currentState = {IDLE, 0};

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
  char MQTT_ID[MQTT_ID_SIZE];
  char USERNAME[MQTT_USERNAME_SIZE];
  char MQTT_PASSWORD[MQTT_PASSWORD_SIZE];
  char PREFIX_TOPIC[PREFIX_SIZE];

  controller.runConfigFile(SSID, PASS, HOST_NAME, IP_ADDRESS, &PORT, MQTT_ID, USERNAME, MQTT_PASSWORD, PREFIX_TOPIC);
  controller.setUpDefaultParameters(stage1_params, stage2_params, stage3_params, room, temp_set);

  start_btn.begin();
  stop_btn.begin();
  d_start_button.begin();

  controller.setUpWiFi(SSID, PASS, HOST_NAME);
  controller.connectToWiFi(/* web_server */ true, /* web_serial */ true, /* OTA */ true);
  controller.setUpRTC();

  StageState last_state = controller.getLastState();
  logger.println("Last state: " + String(last_state.stage) + " " + String(last_state.step));
  if (last_state.stage != IDLE) currentState = last_state;
  
  DateTime current_date = controller.getDateTime();
  logger.println("Current date: " + String(current_date.hour()) + ":" + String(current_date.minute()) + " " + String(current_date.day()) + "/" + String(current_date.month()));

  mqtt.connect(IP_ADDRESS, PORT, MQTT_ID, USERNAME, MQTT_PASSWORD);
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

  // if ((TA) > (LOW_TEMP_LIMIT) && (TA) < (HIGH_TEMP_LIMIT))
   TA_F = TA;  // if the temperature over the limit it will not be considered
  
  // if ((TS) > (LOW_TEMP_LIMIT) && (TS) < (HIGH_TEMP_LIMIT))
   TS_F = TS;
  
  // if ((TC) > (LOW_TEMP_LIMIT) && (TC) < (HIGH_TEMP_LIMIT))
   TC_F = TC;
  
  // if ((TI) > (LOW_TEMP_LIMIT) && (TI) < (HIGH_TEMP_LIMIT))
   TI_F = TI;

  handleInputs();

  handleStage();

  if (hasIntervalPassed(get_temp_timer, TIME_ACQ_DELAY)) publishTemperatures(current_date);  
  
}


bool handleInputs(button_type override) {
  button_type button = NONE;
  bool pressed_btn = false;

  if (override) pressed_btn = button = override;

  if      (d_start_button.released()) button = D_START;
  else if (start_btn.released()) button = START;
  else if (stop_btn.released())  button = STOP;

  if      (button == D_START) setStage(STAGE1);
  else if (button == START)   setStage(STAGE2);
  else if (button == STOP)    stopRoutine(); 

  return !pressed_btn;
}

void handleStage(){
  // logger.print("Current State: "+ String(currentState) + " ");
  if      (currentState.stage == STAGE1)  handleStage1();
  else if (currentState.stage == STAGE2)  handleStage2();
  else if (currentState.stage == STAGE3)  handleStage3();

  else if (currentState.stage == IDLE)    idle();
  else if (currentState.stage == ERROR)   stopRoutine();
}

void handleStage1(){
  asyncLoopSprinkler(sprinkler_1_stg_1_timer, stage1_params.sprinklerOffTime, stage1_params.sprinklerOnTime);

  // Init stage 1
  if (stage_1.getCurrentStep() == 0) {
    stage_1.init();
    logger.print("STAGE #1, current step: ");
    stage_1.nextStep();
  }
  
  // Step #1
  else if (stage_1.getCurrentStep() == 1){
    if (!controller.readDigitalInput(FAN_IO)){
      hasIntervalPassed(fan_1_timer, stage1_params.fanOnTime, true); 

      controller.turnOnFan(true);                                                                                     // Turn ON F1
      logger.println("Stage 1 F1 On");

      publishStateChange(m_F1, true, "Stage 1 init M_F1 ON published ");
    }

    else if (hasIntervalPassed(fan_1_timer, stage1_params.fanOffTime , true)) {
      logger.print("STAGE #1, current step: ");
      stage_1.nextStep();
    }
     
  }

  // Step #2
  else if (stage_1.getCurrentStep() == 2 ){
    // Turn OFF F1 when the time set in the configuration is over
    if (controller.readDigitalInput(FAN_IO)) {
      hasIntervalPassed(fan_1_timer, stage1_params.fanOffTime , true); // In case that the time is over or that the stage and step are not updated

      controller.turnOnFan(false);
      logger.println("Stage 1 F1 Off");

      publishStateChange(m_F1, false, "Stage 1 init M_F1 OFF published ");
    }
    else if (hasIntervalPassed(fan_1_timer, stage1_params.fanOnTime, true)) {
      logger.print("STAGE #1, current step: ");
      stage_1.nextStep();
    }
  }

  // Step #3
  else if (stage_1.getCurrentStep() == 3){
    if (!controller.readDigitalInput(FAN_IO)){
      hasIntervalPassed(fan_1_timer, stage1_params.fanOnTime, true); // In case that the time is over or that the stage and step are not updated

      controller.turnOnFan(true, true); 
      logger.println("Stage 1 F1 On");

      publishStateChange(m_F1, true, "Stage 1 init M_F1 ON published ");
    }

    else if (hasIntervalPassed(fan_1_timer, stage1_params.fanOffTime , true)) {
      logger.print("STAGE #1, current step: ");
      stage_1.nextStep();
    }
  }

  // Step #4
  else if (stage_1.getCurrentStep() == 4){
    if (controller.readDigitalInput(FAN_IO)) {
      hasIntervalPassed(fan_1_timer, stage1_params.fanOffTime , true); // In case that the time is over or that the stage and step are not updated
      
      controller.turnOnFan(false);
      logger.println("Stage 1 F1 Off");

      publishStateChange(m_F1, false, "Stage 1 init M_F1 OFF published ");
    }

    else if (hasIntervalPassed(fan_1_timer, stage1_params.fanOnTime, true)) stage_1.setStep(1);
  }

  DateTime current_date = controller.getDateTime();
  
  bool is_after_stage2_time = current_date.hour() >= stage2_hour && current_date.minute() >= stage2_minute;
  bool is_after_stage_2_date = current_date.day() >= stage2_day && current_date.month() >= stage2_month;

  if (is_after_stage2_time && is_after_stage_2_date){
    stage_1.destroy();
    setStage(STAGE2);
  }
  //  Stage 1 Process 
}

void handleStage2(){
  // Loop of async process
  asyncLoopSprinkler(sprinkler_1_stg_2_timer, stage2_params.sprinklerOffTime, stage2_params.sprinklerOnTime);

  // Init stage 2
  if (stage_2.getCurrentStep() == 0) {
    stage_2.init(); 
    logger.print("STAGE #2, current step: ");
    stage_2.nextStep();
    delay(5000);
  }

  // Step #1
  else if (stage_2.getCurrentStep() == 1){
    if (!controller.readDigitalInput(FAN_IO)) {
      hasIntervalPassed(fan_1_stg_2_timmer, stage2_params.fanOffTime, true);
      controller.turnOnFan(true);// Output of F1
      controller.writeDigitalOutput(AIR_DAMPER_IO, HIGH);  // Air damper
      logger.println("Stage 2 F1 On");

      publishStateChange(m_F1, true, "Stage 2 F1 Start published ");
    }


    // Calculate the Setpoint every 3 seconds in Function of Ta with the formula : Setpoint = A*(B-Ta)
    if (hasIntervalPassed(pid_computing_timer, 3000)) {
      Setpoint = (-(room.A * (temp_data.avg_ts)) + room.B);  //use the average of the temperature over the x last minuites
      pid_setpoint = float(Setpoint);

      logger.println("New Setpoint: " +String(pid_setpoint));


      publishStateChange(SETPOINT, pid_setpoint, "Setpoint published ");
    }

    // Activate the PID when F1 ON
    if (controller.readDigitalInput(FAN_IO) && hasIntervalPassed(turn_on_pid_timer, 3000)) {
      pid_input = TA_F;
      // coef_output = Output;  // Transform the Output of the PID to the desired max value
      coef_output = (coef_pid * Output) / 100;  // Transform the Output of the PID to the desired max value

      air_in_feed_PID.Compute();
      logger.println("Computing PID: " +String(coef_output));
      
      controller.writeAnalogOutput(AIR_PWM, coef_output);
      publishPID();
    }

    // Turn OFF F1 when time is over
    if (controller.readDigitalInput(FAN_IO) && hasIntervalPassed(fan_1_stg_2_timmer, stage2_params.fanOnTime, true)) stage_2.nextStep();
  }

  // Step #2
  else if (stage_2.getCurrentStep() == 2 ){
    if (controller.readDigitalInput(FAN_IO)) {
      hasIntervalPassed(fan_1_stg_2_timmer, stage2_params.fanOnTime, true); // In case that the time is over or that the stage and step are not updated

      controller.turnOnFan(false);
      controller.writeDigitalOutput(AIR_DAMPER_IO, LOW);  // Air damper

      logger.println("Stage 2 F1 Off");
      publishStateChange(m_F1, false, "Stage 2 F1 Stop published ");
    }

    // Put the PID at 0 when F1 OFF
    if (!controller.readDigitalInput(FAN_IO) && hasIntervalPassed(turn_off_pid_timer, 3000)) {
      //Setpoint = 0;
      pid_input = 0;
      Output = 0;
      coef_output = 0;
      
      controller.writeAnalogOutput(AIR_PWM, Output);
      publishPID();
    }

    if (!controller.readDigitalInput(FAN_IO) && hasIntervalPassed(fan_1_stg_2_timmer, stage2_params.fanOffTime, true)) stage_2.nextStep();
  }

  //Step #3
  else if (stage_2.getCurrentStep() == 3){
     if (!controller.readDigitalInput(FAN_IO)){
      hasIntervalPassed(fan_1_stg_2_timmer, stage2_params.fanOffTime, true); // In case that the time is over or that the stage and step are not updated

      controller.turnOnFan(true, true);
      logger.println("Stage 2 F1 On");

      publishStateChange(m_F1, true, "Stage 2 F1 Start published ");
    }

    if (controller.readDigitalInput(FAN_IO) && hasIntervalPassed(fan_1_stg_2_timmer, stage2_params.fanOnTime, true)) stage_2.nextStep();
  }

  //Step #4
  else if (stage_2.getCurrentStep() == 4){
    if (controller.readDigitalInput(FAN_IO)) {
      hasIntervalPassed(fan_1_stg_2_timmer, stage2_params.fanOnTime, true); // In case that the time is over or that the stage and step are not updated

      controller.turnOnFan(false);
      // controller.writeDigitalOutput(AIR_DAMPER_IO, HIGH);  // Air damper
      logger.println("Stage 2 F1 Off");

      publishStateChange(m_F1, false, "Stage 2 F1 Stop published ");
    }

    if (!controller.readDigitalInput(FAN_IO) && hasIntervalPassed(fan_1_stg_2_timmer, stage2_params.fanOffTime, true)) stage_2.setStep(1);
  }

  bool isReadyForStage3 = TS_F >= temp_set.ts && TC_F >= temp_set.tc;

  if (isReadyForStage3 ){
    stage_2.destroy();
    setStage(STAGE3);
  } 
  //  Stage 2 Process
}

void handleStage3(){
  // Loop of async process
  asyncLoopSprinkler(sprinkler_1_stg_3_timer, stage3_params.sprinklerOffTime, stage3_params.sprinklerOnTime);

  // Init stage 3
  if (stage_3.getCurrentStep() == 0) {
    stage_3.init();
    logger.print("STAGE #3, current step: ");
    stage_3.nextStep();
    delay(5000);
  }

  // Step #1
  else if (stage_3.getCurrentStep() == 1){

    if (!controller.readDigitalInput(FAN_IO)) {
      hasIntervalPassed(fan_1_stg_3_timer, stage3_params.fanOffTime, true);
      controller.turnOnFan(true);// Output of F1
      logger.println("Stage 3 F1 On");

      publishStateChange(m_F1, true, "Stage 3 F1 Start published ");
    }

    if(controller.readDigitalInput(FAN_IO) && hasIntervalPassed(fan_1_stg_3_timer, stage3_params.fanOnTime, true)) stage_3.nextStep();
  }

  // Step #2
  else if (stage_3.getCurrentStep() == 2){
    if (controller.readDigitalInput(FAN_IO)) {
      hasIntervalPassed(fan_1_stg_3_timer, stage3_params.fanOnTime, true); // In case that the time is over or that the stage and step are not updated

      controller.turnOnFan(false);
      controller.writeDigitalOutput(AIR_DAMPER_IO, LOW);  // Air damper
      logger.println("Stage 3 F1 Off");

      publishStateChange(m_F1, false, "Stage 3 F1 Stop published ");
    }

    if (!controller.readDigitalInput(FAN_IO) && hasIntervalPassed(fan_1_stg_3_timer, stage3_params.fanOffTime, true)) stage_3.nextStep();
  }

  // Step #3
  else if (stage_3.getCurrentStep() == 3){

    if (!controller.readDigitalInput(FAN_IO)){
      hasIntervalPassed(fan_1_stg_3_timer, stage3_params.fanOffTime, true); // In case that the time is over or that the stage and step are not updated

      controller.turnOnFan(true, true);// Output of F1
      logger.println("Stage 3 F1 On");

      publishStateChange(m_F1, true, "Stage 3 F1 Start published ");
    }

    if (controller.readDigitalInput(FAN_IO) && hasIntervalPassed(fan_1_stg_3_timer, stage3_params.fanOnTime, true)) stage_3.nextStep();
  }

  // Step #4
  else if (stage_3.getCurrentStep() == 4){

    if (controller.readDigitalInput(FAN_IO)) {
      hasIntervalPassed(fan_1_stg_3_timer, stage3_params.fanOnTime, true); // In case that the time is over or that the stage and step are not updated

      controller.turnOnFan(false);
      // controller.writeDigitalOutput(AIR_DAMPER_IO, HIGH);  // Air damper
      logger.println("Stage 3 F1 Off");

      publishStateChange(m_F1, false, "Stage 3 F1 Stop published ");
    }

    if (!controller.readDigitalInput(FAN_IO) && hasIntervalPassed(fan_1_stg_3_timer, stage3_params.fanOffTime, true)) stage_3.setStep(1);
  }

  bool finishedStage3 = false;

  if (finishedStage3){
    stage_3.destroy();
    stopRoutine();
  }
}

void idle(){
  // delay(1000);
}

//// fct Callback ==> RECEIVE MQTT MESSAGES ////////////////////////////////////////////////////////////////////
void callback(char *topic, byte *payload, unsigned int len) {
  logger.println("Message arrived [" + String(topic) + "]" );

   // STOP
  if (mqtt.isTopicEqual(topic, sub_stop) ) {
    const bool value = mqtt.responseToInt(payload, len);
    if (!value) return;

    handleInputs(STOP);
    logger.println("stop BUTTON PRESSED ON NODE RED" );
  }

  // Choose TS
  if (mqtt.isTopicEqual(topic, sub_chooseTs)) {
    is_rts_ir = mqtt.responseToInt(payload, len);
    logger.println("Ts is now IR" + String(is_rts_ir));
  }

  // LoRaTc
  if (mqtt.isTopicEqual(topic, LORA_TC)) {
    if (!controller.isLoraTc()) return;

    const float tc_raw = mqtt.responseToFloat(payload, len);
    TC = isValidTemperature(tc_raw, TC_MIN, TC_MAX, "TC") ? tc_raw : TC_DEF;

    logger.println(String(TC));
  }

  if (mqtt.isTopicEqual(topic, IS_TC_LORA)) {
    controller.setLoraTc(mqtt.responseToInt(payload, len));
    
    logger.println("Lora TC is now: " + String(controller.isLoraTc()));
  }

  // IR_TS
  if (mqtt.isTopicEqual(topic, IS_TS_IR)) {
    controller.setTsContactLess(mqtt.responseToInt(payload, len));

    logger.println("Ts is now: " + String(controller.isTsContactLess()));
  }
  

  if (currentState.stage != IDLE) return;

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
    kp_has_changed = 1;
  }

  if (mqtt.isTopicEqual(topic, sub_I)) {
    Ki = mqtt.responseToFloat(payload, len);
    logger.println("I set to: " + String(Ki));
    ki_has_changed = 1;
  }

  if (mqtt.isTopicEqual(topic, sub_D)) {
    Kd = mqtt.responseToFloat(payload, len);
    logger.println("D set to: " + String(Kd));
    kd_has_changed = 1;
  }

  if (kp_has_changed == 1 && ki_has_changed == 1 && kd_has_changed == 1) {
    air_in_feed_PID.SetTunings(Kp, Ki, Kd);
    logger.println("New PID parameter updated");
    kp_has_changed = ki_has_changed = kd_has_changed = 0;
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

  if(update_default_parameters) controller.updateDefaultParameters(stage1_params, stage2_params, stage3_params, room, temp_set);
}

void stopRoutine() {
  logger.println("PROCESS STOP INITIATED");
  controller.writeDigitalOutput(VALVE_IO, LOW);
  controller.turnOnFan(false);
  controller.writeAnalogOutput(AIR_PWM, 0);

  Output = 0;
  coef_output = 0;

  mqtt.publishData(m_F1, false);
  mqtt.publishData(m_S1, false);
  setStage(IDLE);

  stage_1.destroy();
  stage_2.destroy();
  stage_3.destroy();

  fan_1_timer = 0UL;               // fan F1 timing
  pid_computing_timer = 0UL;    // PID computing timing
  fan_1_stg_2_timmer = 0UL;        // F1 stage 2 timing
  fan_2_stg_2_timmer = 0UL;        // F1 stage 2 timing
  fan_1_stg_3_timer = 0UL;         // F1 stage 3 timing
  sprinkler_1_stg_1_timer = 0UL;         // S1 stage 2 timing
  sprinkler_1_stg_2_timer = 0UL;         // S1 stage 2 timing
  sprinkler_1_stg_3_timer = 0UL;         // S1 stage 3 timing

  logger.println("Stage 0 Status Send packet ");
}

bool isValidTemperature(float temp, float minTemp, float maxTemp, const String& sensorName) {
  bool is_valid = temp >= minTemp && temp <= maxTemp;
  if(is_valid) sendTemperaturaAlert(temp, sensorName);

  return is_valid;
}

void updateTemperature() {
  float ta_raw = controller.readTempFrom(TA_AI);
  float tc_raw = controller.readTempFrom(TC_AI);
  float ts_raw = controller.isTsContactLess() ? controller.getIRTemp() : controller.readTempFrom(TS_AI);
  
  
  TA = isValidTemperature(ta_raw, TA_MIN, TA_MAX, "TA") ? ta_raw : TA_DEF;
  if (!controller.isLoraTc()) TC = isValidTemperature(tc_raw, TC_MIN, TC_MAX, "TC") ? tc_raw : TC_DEF;
  TS = isValidTemperature(ts_raw, TS_MIN, TS_MAX, "TS") ? ts_raw : TS_DEF;

  getTempAvg();
}

void sendTemperaturaAlert(float temp, String sensor){
  const String msg = "{\"temp\":" + String(temp) + ", \"sensor\":" + sensor + "}";
  mqtt.publishData(SPOILED_SENSOR, msg);
}

void setStage(SystemState stage) {
  currentState.stage = stage;
  currentState.step = 0;

  mqtt.publishData(STAGE, stage);
  controller.saveLastState(currentState);

  for (int i = STAGE1; i <= STAGE3; i++) {
    if (stageLedPins[i] != -1) digitalWrite(stageLedPins[i], LOW);
  }

  if (stage >= STAGE1 && stage <= STAGE3 && stageLedPins[stage] != -1) {
      digitalWrite(stageLedPins[stage], HIGH);
  }
}

bool hasIntervalPassed(uint32_t &previousMillis, uint32_t interval, bool to_min) {
  unsigned long currentMillis = millis();
  if(to_min) interval *= 60000;
  if (previousMillis == 0) {
    previousMillis = millis() - interval;
  }

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
  mqtt.publishData(STAGE, currentState.stage);

  mqtt.publishData(m_F1, controller.readDigitalInput(FAN_IO));
  mqtt.publishData(m_F1_CCW, controller.readDigitalInput(FAN_CCW_IO));

  mqtt.publishData(m_S1, controller.readDigitalInput(VALVE_IO));

  mqtt.publishData(SETPOINT, Setpoint);

  publishTemperatures();
  publishPID();
}

void getTempAvg() {
  sensorTs.addValue(TS_F); // Añadir el valor al buffer de Ts
  sensorTc.addValue(TC_F);
  sensorTa.addValue(TA_F);

  // mqtt.publishData(AVG_TS_TOPIC, temp_data.avg_ts);
  // mqtt.publishData(AVG_TC_TOPIC, temp_data.avg_tc);
  // mqtt.publishData(AVG_TA_TOPIC, temp_data.avg_ta);
}

void publishPID(){
  if (hasIntervalPassed(stg_2_pid_timer, TIME_ACQ_DELAY + 1)) {
    // logger.println("Coef Output Actual Output is " + String(coef_output));
    const float output_float = float(coef_output);
    pid_output = ((output_float - 0) / (255 - 0)) * (100 - 0) + 0;

    publishStateChange(PID_OUTPUT, pid_output,"PID Output /100 is " + String(pid_output));
  }
}

void publishTemperatures(DateTime &current_date) {
// void publishTemperatures() {
  temp_data.ta = TA_F;
  temp_data.ts = TS_F;
  temp_data.tc = TC_F;
  temp_data.ti = TI_F;
  temp_data.avg_ts = sensorTs.getAverage();
  temp_data.avg_tc = sensorTc.getAverage();
  temp_data.avg_ta = sensorTa.getAverage();


  mqtt.publishData(TA_TOPIC, temp_data.ta);
  mqtt.publishData(TS_TOPIC, temp_data.ts);
  mqtt.publishData(TC_TOPIC, temp_data.tc);
  mqtt.publishData(TI_TOPIC, temp_data.ti);

  mqtt.publishData(AVG_TA_TOPIC, temp_data.avg_ta);
  mqtt.publishData(AVG_TS_TOPIC, temp_data.avg_ts);
  mqtt.publishData(AVG_TC_TOPIC, temp_data.avg_tc);

  // for debug purpose
  // logger.println("Average: " + String(temp_data.avg_ts));
  logger.println("Ts: " + String(TS));
  logger.println("TC: " + String(TC));
  logger.println("Ta: " + String(TA));
  // // logger.println("Nstart: " + String(remote_start));
  // // logger.println("Nstop: " + String(remote_stop));
  // logger.println("A variable: " + String(room.A));
  // logger.println("B variable: " + String(room.B));
  // logger.println("P variable: " + String(Kp));
  // logger.println("I variable: " + String(Ki));
  // logger.println("D variable: " + String(Kd));
  // logger.println("setpoint raw: " + String(Setpoint));
  // logger.println("setpoint: " + String(pid_setpoint));

  // logger.printTime("Time:", current_date.hour(), current_date.minute(), current_date.day(), current_date.month());
  // logger.printTime("Stage 2 Time:", stage2_hour, stage2_minute, stage2_day, stage2_month);
}

void initStage1(){
  logger.println("Stage 1 Initiated");

  publishStateChange(m_F1, false, "Stage 1 init M_F1 stop published ");
  publishStateChange(m_F2, false, "Stage 1 init M_F2 stop published ");
  publishStateChange(m_S1, false, "Stage 1 init M_S1 stop published");

  controller.writeDigitalOutput(VALVE_IO, LOW);
  controller.turnOnFan(false);
}

void initStage2(){
  logger.println("Stage 2 Initiated");
  
  publishStateChange(m_F1, false, "Stage 2 init M_F1 stop published ");
  publishStateChange(m_F2, false, "Stage 2 init M_F2 stop published ");
  publishStateChange(m_S1, false, "Stage 2 init M_S1 stop published");
}

void initStage3(){
  logger.println("Stage 3 Initiated");

  publishStateChange(m_F2, false, "Stage 3 M_F2 init published ");
  publishStateChange(m_F1, false, "Stage 3 F1 init published ");
  publishStateChange(m_S1, false, "Stage 3 S1 init published ");
}

void destroyStage1(){
  logger.println("Stage 1 Destroyed");

  controller.writeDigitalOutput(VALVE_IO, LOW);
  controller.turnOnFan(false);
}

void destroyStage2(){
  logger.println("Stage 2 Destroyed");

  Output = 0;
  coef_output = 0;

  controller.writeAnalogOutput(AIR_PWM, 0);
  controller.writeDigitalOutput(VALVE_IO, LOW);

  controller.turnOnFan(false);
}

void destroyStage3(){
  logger.println("Stage 3 Destroyed");

  controller.writeAnalogOutput(AIR_PWM, 0);
  controller.writeDigitalOutput(VALVE_IO, LOW);
  controller.turnOnFan(false);

}

void onMQTTConnect() {
  mqtt.publishData(m_F1, controller.readDigitalInput(FAN_IO));
  // mqtt.publishData(m_F2, fan_2);
  mqtt.publishData(m_S1, controller.readDigitalInput(VALVE_IO));
  mqtt.publishData(STAGE, currentState.stage);
  mqtt.publishData(TA_TOPIC, TA);
  mqtt.publishData(TS_TOPIC, TS);
  mqtt.publishData(TC_TOPIC, TC);
  mqtt.publishData(TI_TOPIC, TI);
  mqtt.publishData(PID_OUTPUT, pid_output);
  mqtt.publishData(SETPOINT, Setpoint);
}

void asyncLoopSprinkler(uint32_t &timer, uint32_t offTime, uint32_t onTime) {
  bool isSprinklerOn = controller.readDigitalInput(VALVE_IO);

  if (!isSprinklerOn && hasIntervalPassed(timer, offTime, true)) {
    // Es tiempo de encender el aspersor
    controller.writeDigitalOutput(VALVE_IO, HIGH);
    logger.println("Sprinkler ON at: " + String(millis()));
    
    publishStateChange(m_S1, true, "Stage 2 S1 Start published "); 
  } 
  
  else if (isSprinklerOn && hasIntervalPassed(timer, onTime, true)) {
    // Es tiempo de apagar el aspersor
    controller.writeDigitalOutput(VALVE_IO, LOW);
    logger.println("Sprinkler OFF at: " + String(millis()));

    publishStateChange(m_S1, false, "Stage 2 S1 Stop published ");
  }
}