#include "Thawing-room-chicano.h"

data_rtc N_rtc;  // structure data_rtc from the config file is renamed N_rtc

// Stage parameters
stage_parameters stage1_params;
stage_parameters stage2_params;  // fan (F1) and sprinklers (S1) STAGE 2 on and off time
stage_parameters stage3_params;

// A & B variables
room_parameters room;

// State of SPRINKLER 1
bool S1_state = 0;

// PID variables
data_PIDO PID_data;           // value of the PID output
data_setpoint setpoint_data;  // value of the Setpoint

bool R_P = 0,  R_I = 0, R_D = 0;

double Output;    // PWM signal and converter
double PIDinput;  // temp sensor
double Setpoint;  // will be the desired value

// PID parameters
double Kp = 0, Ki = 10, Kd = 0;

double coefOutput = 0;  // Output for the infeed (New Analog Output that will be sent to S1
uint8_t coefPID = 100;
float Output_float = 0.0;
uint8_t Converted_Output = 0;

// Temperatures measures of Ta, Ts, Tc, Ti & AvgTs
data_s temp_data;

// Ts & Tc target value
data_tset temp_set;
bool stop_temp1 = 0, stop_temp2 = 0;

// Fan F1 value (1 parameter)
data_F1 F1_data;
data_F2 F2_data;

// Sprinkler S1 value (1 parameter)
data_S1 S1_data;

// Start, delayed start, stop, and choose Ts
uint8_t N_stop = 0;
uint8_t N_start = 0;
uint8_t N_d_start = 0;
uint8_t N_chooseTs = 0;

bool STOP = 0;
bool START = 0;

bool START1 = 0;     // delayed start bttn
bool START2 = 0;     // start bttn
bool C1_state = 0;   // State of Stage 1
bool C2_state = 0;   // State of Stage 2
bool C3_state = 0;   // State of Stage 3
bool MTR_State = 0;  // State of the motor that control the Fan F1
bool MTR2_State = 0; // State of the motor that control the Fan F2

// State of the Stage (data = 1, 2 or 3)
StageState currentState = {IDLE, 0};

Button start_btn(START_IO); // Connect your button between pin 2 and GND
Button stop_btn(STOP_IO); // Connect your button between pin 3 and GND
Button d_start_button(DLY_S_IO); // Connect your button between pin 4 and GND

// Parameters of Stage 2
uint8_t Stage2_hour = 0;
uint8_t Stage2_minute = 0;
uint8_t Stage2_day = 0;
uint8_t Stage2_month = 0;

bool Stage2_RTC_set = 0;
bool Stage2_started = 0;
bool Stage3_started = 0;

// ########################### Timers ##########################
uint32_t F1_timer = 0UL;               // fan F1 timing
uint32_t pid_computing_timer = 0UL;    // PID computing timing
uint32_t F1_stg_2_timmer = 0UL;        // F1 stage 2 timing
uint32_t F2_stg_2_timmer = 0UL;        // F2 stage 2 timing
uint32_t S1_stg_2_timer = 0UL;         // S1 stage 2 timing
uint32_t F1_stg_3_timer = 0UL;         // F1 stage 3 timing
uint32_t S1_stg_3_timer = 0UL;         // S1 stage 3 timing
uint32_t get_temp_timer = 0UL;         // temperature acquisition
uint32_t temp_avg_timer = 0UL;           // Ts average timing
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
SensorBuffer sensorTc(BUFFER_SIZE);  // Crear otra instancia para el sensor Tc
SensorBuffer sensorTa(BUFFER_SIZE_TA); // Ta with a Buffer of

MqttClient mqtt;
Controller controller;
TaskHandle_t communicationTask;
PID air_in_feed_PID(&PIDinput, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  // DIRECT or REVERSE

void backgroundTasks(void* pvParameters) {
  for (;;) {
    controller.WiFiLoop();

    if(controller.isWiFiConnected()) {
      mqtt.loop();
      controller.loopOTA();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
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

  controller.setUpWiFi(SSID, PASS, HOST_NAME);
  controller.connectToWiFi(/* web_server */ true, /* web_serial */ true, /* OTA */ true);
  controller.setUpRTC();

  mqtt.connect(IP_ADDRESS, PORT, MQTT_ID, USERNAME, MQTT_PASSWORD);
  mqtt.setCallback(callback);

  start_btn.begin();
  stop_btn.begin();
  d_start_button.begin();

  // xTaskCreatePinnedToCore(backgroundTasks, "communicationTask", 10000, NULL, 1, &communicationTask, 0);
  //Turn the PID on
  air_in_feed_PID.SetMode(AUTOMATIC);
  air_in_feed_PID.SetSampleTime(3000);
  //Adjust PID values
  air_in_feed_PID.SetTunings(Kp, Ki, Kd);
  
  WebSerial.println("===========> Reboted!! <===========");
  
  StageState last_state = controller.getLastState();
  WebSerial.println("Last state: " + String(last_state.stage) + " " + String(last_state.step));
  if (last_state.stage != IDLE) {
    currentState = last_state;
    handleInputs(currentState.stage);
  }

  delay(750);
}

void loop() {
  // if is for testing porpuse comment this "if" and replace DateTime "now" for: DateTime now(__DATE__, __TIME__); 

  if (!controller.isRTCConnected()) {  
    WebSerial.println("RTC not connected"); 
    while (true) delay(1000);
  }

  DateTime now = controller.getDateTime();
  // DateTime now(__DATE__, __TIME__); 

  controller.WiFiLoop();

  if(controller.isWiFiConnected()) {
    mqtt.loop();
    controller.loopOTA();
  }

  updateTemperature();

  // if (N_chooseTs == 1) TC2 = analogRead(A0);                                // Condition to choose if Ts is a IR sensor or OneWire sensor

  if ((TA) > (LOW_TEMP_LIMIT) && (TA) < (HIGH_TEMP_LIMIT)) TA_F = TA;  // if the temperature over the limit it will not be considered

  if ((TS) > (LOW_TEMP_LIMIT) && (TS) < (HIGH_TEMP_LIMIT)) TS_F = TS;

  if ((TC) > (LOW_TEMP_LIMIT) && (TC) < (HIGH_TEMP_LIMIT)) TC_F = TC;

  if ((TI) > (LOW_TEMP_LIMIT) && (TI) < (HIGH_TEMP_LIMIT)) TI_F = TI;

  if ((millis() - address_sending_timer >= 10000)) {

    String ta_string_address = addressToString(controller.ADDRESS_TA);
    mqtt.publishData("mduino/sendadd1", ta_string_address);

    String ts_string_address = addressToString(controller.ADDRESS_TS);
    mqtt.publishData("mduino/sendadd2", ts_string_address);

    String tc_string_address = addressToString(controller.ADDRESS_TC);
    mqtt.publishData("mduino/sendadd3", tc_string_address);

    String ti_string_address = addressToString(controller.ADDRESS_TI);
    mqtt.publishData("mduino/sendadd4", ti_string_address);

    address_sending_timer = millis();
  }

  //---- Get surface temperature average with a FIFO buffer ---- //////////////////////////////// Something fuckin' wrong with the average
 
    // ------------ Average Ts ---------------//
  if (millis() - temp_avg_timer >= AVG_RESOLUTION) {
    
    sensorTs.addValue(TS_F); // Añadir el valor al buffer de Ts
    sensorTc.addValue(TC_F);
    sensorTa.addValue(TA_F);

    mqtt.publishData(AVG_TS_TOPIC, temp_data.AvgTs_N);
    mqtt.publishData(AVG_TC_TOPIC, temp_data.AvgTc_N);
    mqtt.publishData(AVG_TA_TOPIC, temp_data.AvgTa_N);

    temp_avg_timer = millis();
  }

  //---- Temperature MQTT publish ----///////////////////////////////////////////////////////////
  if (millis() - get_temp_timer >= TIME_ACQ_DELAY) {
    temp_data.Ta_N = TA_F;
    temp_data.Ts_N = TS_F;
    temp_data.Tc_N = TC_F;
    temp_data.Ti_N = TI_F;
    temp_data.AvgTs_N = sensorTs.getAverage();
    temp_data.AvgTc_N = sensorTc.getAverage();
    temp_data.AvgTa_N = sensorTa.getAverage();

    mqtt.publishData(TA_TOPIC, temp_data.Ta_N);
    mqtt.publishData(TS_TOPIC, temp_data.Ts_N);
    mqtt.publishData(TC_TOPIC, temp_data.Tc_N);
    mqtt.publishData(TI_TOPIC, temp_data.Ti_N);

    // for debug purpose
    WebSerial.println("Stage : " + String(currentState.stage));
    WebSerial.println("Average: " + String(temp_data.AvgTs_N));
    // WebSerial.println(controller.readDigitalInput(DI0));
    WebSerial.println("Ta: " + String(TA));
    WebSerial.println("Ts: " + String(TS));
    WebSerial.println("TC: " + String(TC));
    WebSerial.println("Ti: " + String(TI));
    // WebSerial.println(controller.readAnalogInput(TI_AI));   // <=========== this shit it's da problem 
    WebSerial.println("Nstart: " + String(N_start));
    WebSerial.println("Nstop: " + String(N_stop));
    WebSerial.println("A variable: " + String(room.A));
    WebSerial.println("B variable: " + String(room.B));
    WebSerial.println("P variable: " + String(Kp));
    WebSerial.println("I variable: " + String(Ki));
    WebSerial.println("D variable: " + String(Kd));
    WebSerial.println("setpoint raw: " + String(Setpoint));
    WebSerial.println("setpoint: " + String(setpoint_data.PID_setpoint));

    WebSerial.println("time: ");
    WebSerial.print(String(now.hour()) + "h ");
    WebSerial.println(String(now.minute()) + "min ");
    WebSerial.print(String(now.day()) + "day ");
    WebSerial.println(String(now.month()) + "month");
    WebSerial.println("stage 2 time: ");
    WebSerial.print(String(Stage2_hour) + "h ");
    WebSerial.print(String(Stage2_minute) + "min ");
    WebSerial.print(String(Stage2_day) + "day ");
    WebSerial.print(String(Stage2_month) + "month");

    get_temp_timer = millis();
  }

  //---- Time Stage ON/OFF and A & B MQTT Publish ----///////////////////////////////////////////////////////
  if (millis() - A_B_timer >= 10000) {
    mqtt.publishData(STAGE, currentState.stage);
    mqtt.publishData(m_F1, F1_data.M_F1);
    // mqtt.publishData(m_F2, fan_2);
    mqtt.publishData(m_S1, S1_data.M_S1);
    mqtt.publishData(PID_OUTPUT, coefOutput);
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

    // Ts & Tc
    mqtt.publishData(ACK_TS, temp_set.ts);
    mqtt.publishData(ACK_TC, temp_set.tc);

    A_B_timer = millis();
  }

  //---- PID Publishing ----//////////////////////////////////////////////////////////////////////
  // PID works only on STAGE 2

  if (millis() - stg_2_pid_timer >= (TIME_ACQ_DELAY + 1)) {
    WebSerial.println("Soft PID Actual Output is" + String(Output));
    Output_float = float(coefOutput);
    PID_data.PID_output = ((Output_float - 0) / (255 - 0)) * (100 - 0) + 0;
    WebSerial.println("PID Output /100 is" + String(PID_data.PID_output));

    mqtt.publishData(PID_OUTPUT, PID_data.PID_output);
    stg_2_pid_timer = millis();
  }
  
  handleInputs();

  //---- STOP ROUTINE ----///////////////////////////////////////////////////////////////////////
  if (STOP == 1) stopRoutine();
  //---- RTC Timer ----//////////////////////////////////////////////////////////////////////////

  if (((((now.hour() >= Stage2_hour && now.minute() >= Stage2_minute
          && now.day() >= Stage2_day && now.month() >= Stage2_month)
         && START1 == 1)
        || START2 == 1)
       && Stage2_started == 0 && Stage2_RTC_set == 0)) {

    START1 = MTR_State = MTR2_State = C1_state = 0;
    
    controller.writeDigitalOutput(STAGE_1_IO, LOW);
    controller.writeDigitalOutput(STAGE_2_IO, LOW);
    controller.writeDigitalOutput(STAGE_3_IO, LOW);
    controller.writeDigitalOutput(VALVE_IO, LOW);
    controller.writeDigitalOutput(FAN_IO, LOW);
    controller.writeDigitalOutput(FAN2_IO, LOW);

    F1_data.M_F1 = 2;
    F2_data.M_F2 = 0;
    S1_data.M_S1 = 2;

    mqtt.publishData(m_F1, F1_data.M_F1);
    WebSerial.println("All M_F1 stop published ");

    mqtt.publishData(m_F2, F2_data.M_F2);
    WebSerial.println("ALL M_F2 init M_F2 stop published ");

    mqtt.publishData(m_S1, S1_data.M_S1);
    WebSerial.println("All M_S1 stop published");

    WebSerial.println("Stage 2 Initiated wait for 5 secs");
    Stage2_RTC_set = Stage2_started = 1;
    delay(5000);
  }

  //---- STAGE 1 ----////////////////////////////////////////////////////////////////////////////
  if (START1 == 1 && Stage2_RTC_set == 0 && STOP == 0) {
    if (C1_state == 0) {
      controller.writeDigitalOutput(STAGE_1_IO, HIGH);  // Turn On the LED of Stage 1
      C1_state = 1;                    // State of Stage 1 turned ON
      WebSerial.println("Stage 1 Started");
      setStage(STAGE1);
      WebSerial.println("Stage 1 Status Send packet ");
      F1_timer = millis() - (stage1_params.fanOnTime * MINS);
    }

    // Turn ON F1

    if (MTR_State == 0 && (HIGH != controller.readDigitalInput(FAN_IO)) && (millis() - F1_timer >= (stage1_params.fanOffTime * MINS))) {  // MTR_State is the motor of F1
      controller.writeDigitalOutput(FAN_IO, HIGH);                                                                                       // Turn ON F1
      WebSerial.println("Stage 1 F1 On");
      MTR_State = 1;
      F1_data.M_F1 = 1;  // When M_F1 = 1 ==> ON

      mqtt.publishData(m_F1, F1_data.M_F1);
      WebSerial.println("Stage 1 init M_F1 ON published ");
      F1_timer = millis();
    }

    // Turn OFF F1 when the time set in the configuration is over
    if (MTR_State == 1 && (LOW != controller.readDigitalInput(FAN_IO)) && (millis() - F1_timer >= (stage1_params.fanOnTime * MINS))) {
      controller.writeDigitalOutput(FAN_IO, LOW);
      controller.writeAnalogOutput(AIR_PWM, 0); // for chicano output is inverted 255=0V and 0=10V
      WebSerial.println("Stage 1 F1 Off");
      MTR_State = 0;
      F1_data.M_F1 = 2;  // When M_F1 = 2 ==> OFF

      mqtt.publishData(m_F1, F1_data.M_F1);
      WebSerial.println("Stage 1 init M_F1 OFF published ");
      F1_timer = millis();
    }
    
  }

  //---- STAGE 2 ----////////////////////////////////////////////////////////////////////////////
  if (Stage2_RTC_set == 1 && Stage3_started == 0 && STOP == 0) {
    if (C2_state == 0) {
      controller.writeDigitalOutput(STAGE_2_IO, HIGH);  // Turn On the LED of Stage 2

      C2_state = 1;
      WebSerial.println("Stage 2 Started");
      setStage(STAGE2);
      WebSerial.println("Stage 0 Status Send packet ");
      F1_stg_2_timmer = millis() - (stage2_params.fanOffTime * MINS);
    }

    // Turn ON F1 when time is over
    if (MTR_State == 0 && (millis() - F1_stg_2_timmer >= (stage2_params.fanOffTime * MINS))) {
      controller.writeDigitalOutput(FAN_IO, HIGH);  // Output of F1
      WebSerial.println("Stage 2 F1 On");
      MTR_State = 1;
      F1_data.M_F1 = 1;  // When M_F1 = 1 ==> ON

      mqtt.publishData(m_F1, F1_data.M_F1);
      WebSerial.println("stg2 F1 Start published ");
      F1_stg_2_timmer = millis();
    }

    // Turn OFF F1 when time is overcommunicationTask
    if (MTR_State == 1 && (millis() - F1_stg_2_timmer >= (stage2_params.fanOnTime * MINS))) {
      controller.writeDigitalOutput(FAN_IO, LOW);
      WebSerial.println("Stage 2 F1 Off");
      MTR_State = 0;
      F1_data.M_F1 = 2;  // When M_F1 = 2 ==> OFF

      mqtt.publishData(m_F1, F1_data.M_F1);
      WebSerial.println("stg2 F1 stop published ");
      F1_stg_2_timmer = millis();
    }

    asyncLoopSprinkler(S1_stg_2_timer, stage2_params.sprinklerOffTime , stage2_params.sprinklerOnTime);
    // Turn ON S1 when time is over
//     if ((MTR_State == 1) && (S1_state == 0) && (millis() - S1_stg_2_timer >= (stage2_params.sprinklerOffTime * MINS))) {   // ===== DONE =====
//       controller.writeDigitalOutput(VALVE_IO, HIGH);  // Output of S1
//       S1_state = 1;
//       WebSerial.println("Stage 2 S1 ON");
//       S1_data.M_S1 = 1;  // When M_S1 = 1 ==> ON
// >>>>>>> refs/remotes/origin/MFP-V2

    // // Turn ON S1 when time is over
    // if ((MTR_State == 1) && (S1_state == 0) && (millis() - S1_stg_2_timer >= (stage2_params.sprinklerOffTime * MINS))) {
    //   controller.writeDigitalOutput(VALVE_IO, HIGH);  // Output of S1
    //   S1_state = 1;
    //   WebSerial.println("Stage 2 S1 ON");
    //   S1_data.M_S1 = 1;  // When M_S1 = 1 ==> ON

    //   mqtt.publishData(m_S1, S1_data.M_S1);
    //   WebSerial.println("stg2 S1 start published");
    //   S1_stg_2_timer = millis();
    // }

    // // Turn OFF S1 when time is over
    // if ((S1_state == 1 && (millis() - S1_stg_2_timer >= (stage2_params.sprinklerOnTime * MINS))) || (MTR_State == 0)) {
    //   controller.writeDigitalOutput(VALVE_IO, LOW);  // Output of S1
    //   S1_state = 0;
    //   WebSerial.println("Stage 2 S1 OFF");
    //   S1_data.M_S1 = 2;  // When M_S1 = 2 ==> OFF

    //   mqtt.publishData(m_S1, S1_data.M_S1);
    //   WebSerial.println("stg2 S1 stop published");

    //   S1_stg_2_timer = millis();
    // }

    // Calculate the Setpoint every 3 seconds in Function of Ta with the formula : Setpoint = A*(B-Ta)
    if ((millis() - pid_computing_timer >= 3000)) {
      Setpoint = (-(room.A * (temp_data.AvgTs_N)) + room.B);  //use the average of the temperature over the x last minuites
      setpoint_data.PID_setpoint = float(Setpoint);

      mqtt.publishData(SETPOINT, setpoint_data.PID_setpoint);

      WebSerial.println("Setpoint published");
      pid_computing_timer = millis();
    }

    // Activate the PID when F1 ON
    if (MTR_State && (millis() - turn_on_pid_timer >= 3000)) {
      PIDinput = sensorTa.getAverage();
      coefOutput = (coefPID * Output) / 100;
      WebSerial.println(coefOutput);
      air_in_feed_PID.Compute();
      // controller.writeAnalogOutput(AIR_PWM, Output);
      controller.writeAnalogOutput(AIR_PWM, coefOutput);
      Converted_Output = ((Output - 0) / (255 - 0)) * (10000 - 0) + 0;
      WebSerial.println("Converted_Output is " + String(Converted_Output));
      turn_on_pid_timer = millis();
    }

    // Put the PID at 0 when F1 OFF
    if (MTR_State == 0 && (millis() - turn_off_pid_timer >= 3000)) {
      //Setpoint = 0;
      PIDinput = 0;
      Output = 0;
      coefOutput = 0; // inverted on chicano so 255=0V
      // controller.writeAnalogOutput(AIR_PWM, Output);
      controller.writeAnalogOutput(AIR_PWM, coefOutput);
      Converted_Output = ((Output - 0) / (255 - 0)) * (10000 - 0) + 0;
      WebSerial.println("Converted_Output is " + String(Converted_Output));
      turn_off_pid_timer = millis();
    }

    if ((MTR_State == 1 && MTR2_State == 0 && temp_data.Ta_N < (Setpoint - 2)) && (millis() - F2_stg_2_timmer >= 3000)){
      controller.writeDigitalOutput(FAN2_IO, HIGH);
      WebSerial.println("Stage 2 F2 ON");
      MTR2_State = 1;
      F2_data.M_F2 = 1;  // When M_F1 = 2 ==> OFF

      mqtt.publishData(m_F2, F2_data.M_F2);
      WebSerial.println("stg2 F2 on published ");
      F2_stg_2_timmer = millis();
    }

    if ((MTR2_State == 1 && temp_data.Ta_N > (Setpoint + 2)) || (MTR_State == 0 && (millis() - F2_stg_2_timmer >= 3000))){
      controller.writeDigitalOutput(FAN2_IO, LOW);
      WebSerial.println("Stage 2 F2 OFF");
      MTR2_State = 0;
      F2_data.M_F2 = 0;  // When M_F1 = 2 ==> OFF

      mqtt.publishData(m_F2, F2_data.M_F2);
      WebSerial.println("stg2 F2 OFF published ");
      F2_stg_2_timmer = millis();
    }
  }

  //---- STAGE 3 ----////////////////////////////////////////////////////////////////////////////
  // Initialisation Stage3 (reset all the other stages to 0)
  if (sensorTs.getAverage() >= temp_set.ts && sensorTc.getAverage() >= temp_set.tc && Stage3_started == 0 && Stage2_started == 1) {
    START1 = START2 = Stage2_RTC_set = MTR_State = 0;

    // Turn All Output OFF
    controller.writeAnalogOutput(AIR_PWM, 0); // inverted on chicano so 255=0V
    controller.writeDigitalOutput(STAGE_1_IO, LOW);
    controller.writeDigitalOutput(STAGE_2_IO, LOW);
    controller.writeDigitalOutput(STAGE_3_IO, LOW);
    controller.writeDigitalOutput(VALVE_IO, LOW);
    controller.writeDigitalOutput(FAN_IO, LOW);
    controller.writeDigitalOutput(FAN2_IO, LOW);

    Output = 0;
    coefOutput = 0; //inverted on chicano so 255=0V

    F1_data.M_F1 = 2;  // When M_F1 = 2 ==> OFF
    F2_data.M_F2 = 0; // // When M_F2 = 0 ==> OFF

    mqtt.publishData(m_F1, F1_data.M_F1);
    WebSerial.println("stage 3 F1 init published ");

    mqtt.publishData(m_F2, F2_data.M_F2);
    WebSerial.println("Stage 3 M_F2 init published ");

    S1_data.M_S1 = 2;  // When M_S1 = 2 ==> OFF

    mqtt.publishData(m_S1, S1_data.M_S1);
    WebSerial.println("stage 2 S1 init published");

    C2_state = S1_state = 0;  // Put the all the states to 0
    WebSerial.println("Stage 3 Initiated");
    Stage3_started = 1;
  }

  // Stage 3
  if (Stage3_started == 1 && Stage2_started == 1 && STOP == 0) {
    // State of Stage 3 turned to 1
    if (C3_state == 0) {
      controller.writeDigitalOutput(STAGE_3_IO, HIGH);  // Turn ON the LED of Stage 3

      C3_state = 1;
      WebSerial.println("Stage 3 Started");
      setStage(STAGE3);
      WebSerial.println("Stage 3 Status Send packet ");
      F1_stg_3_timer = millis() - (stage3_params.fanOffTime * MINS);
    }

    // Turn ON F1 when time is over
    if (MTR_State == 0 && (millis() - F1_stg_3_timer >= (stage3_params.fanOffTime * MINS))) {
      controller.writeDigitalOutput(FAN_IO, HIGH);
      
      WebSerial.println("Stage 3 F1 On");
      MTR_State = 1;
      F1_data.M_F1 = 1;

      mqtt.publishData(m_F1, F1_data.M_F1);
      WebSerial.println("stage 3 F1 start published ");
      F1_stg_3_timer = millis();
    }

    // Turn OFF F1 when time is over
    if (MTR_State == 1 && (millis() - F1_stg_3_timer >= (stage3_params.fanOnTime * MINS))) {
      controller.writeDigitalOutput(FAN_IO, LOW);
      
      WebSerial.println("Stage 3 F1 Off");
      MTR_State = 0;
      F1_data.M_F1 = 2;

      mqtt.publishData(m_F1, F1_data.M_F1);
      WebSerial.println("stage 3 F1 stop published ");
      F1_stg_3_timer = millis();
    }

    asyncLoopSprinkler(S1_stg_3_timer, stage3_params.sprinklerOffTime , stage3_params.sprinklerOnTime);

    // if (S1_state == 0 && (millis() - S1_stg_3_timer >= (stage3_params.sprinklerOffTime * MINS))) {
    //   controller.writeDigitalOutput(VALVE_IO, HIGH);
    //   S1_state = 1;
    //   WebSerial.println("Stage 3 S1 ON");
    //   S1_data.M_S1 = 1;

    //   mqtt.publishData(m_S1, S1_data.M_S1);
    //   WebSerial.println("stg3 S1 start published");
    //   S1_stg_3_timer = millis();
    // }

    // if (S1_state == 1 && (millis() - S1_stg_3_timer >= (stage3_params.sprinklerOnTime * MINS))) {
    //   controller.writeDigitalOutput(VALVE_IO, LOW);
    //   S1_state = 0;
    //   WebSerial.println("Stage 3 S1 OFF with value of S1 ");
    //   S1_data.M_S1 = 2;

    //   mqtt.publishData(m_S1, S1_data.M_S1);
    //   WebSerial.println("stg3 S1 stop published");
    //   S1_stg_3_timer = millis();
    // }
    
  }
}

void handleInputs(SystemState stage_to_init) {
    //---- START, DELAYED, STOP Button pressed ----////////////////////////////////////////////////
  // delayed start push button or digital button pressed
  if (d_start_button.released() || N_d_start == 1 || stage_to_init == STAGE1) {
    START1 = 1;
    WebSerial.println("Delayed Start Pressed");
    N_d_start = 0;
    F1_data.M_F1 = 2;
    F2_data.M_F2 = 0;
    S1_data.M_S1 = 2;

    mqtt.publishData(m_F1, F1_data.M_F1);
    WebSerial.println("Stage 1 init M_F1 stop published ");

    mqtt.publishData(m_F2, F2_data.M_F2);
    WebSerial.println("Stage 1 init M_F2 stop published ");

    mqtt.publishData(m_S1, S1_data.M_S1);
    WebSerial.println("Stage 1 init M_S1 stop published");
  }

  // start push button or digital button pressed
  if (start_btn.released() || N_start == 1 || stage_to_init == STAGE2) {
    START2 = 1;
    WebSerial.println("Start Pressed");
    N_start = 0;
    F1_data.M_F1 = 2;
    F2_data.M_F2 = 0;
    S1_data.M_S1 = 2;

    mqtt.publishData(m_F1, F1_data.M_F1);
    WebSerial.println("Stage 1 init M_F1 stop published ");

    mqtt.publishData(m_F2, F2_data.M_F2);
    WebSerial.println("Stage 1 init M_F2 stop published ");

    mqtt.publishData(m_S1, S1_data.M_S1);
    WebSerial.println("Stage 1 init M_S1 stop published");
  }

  // stop push button or digital button pressed
  if (stop_btn.released() || N_stop == 1) {
    STOP = 1;
    WebSerial.println("Stop Pressed");
    N_stop = 0;
  }

}

//// fct Callback ==> RECEIVE MQTT MESSAGES ////////////////////////////////////////////////////////////////////
void callback(char *topic, byte *payload, unsigned int len) {
  WebSerial.println("Message arrived [" + String(topic) + "]");

  // Delayed start timing
  if (strcmp(topic, sub_hours) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Stage2_hour = responseToFloat(payload, len);
    WebSerial.println("Stage 2 Hours set to: " + String(Stage2_minute));
  }

  if (strcmp(topic, sub_minutes) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Stage2_minute = responseToFloat(payload, len);
    WebSerial.println("Stage 2 Minutes set to: " + String(Stage2_minute));
  }

  if (strcmp(topic, sub_day) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Stage2_day = responseToFloat(payload, len);
    WebSerial.println("Stage 2 Day set to: " + String(Stage2_day));
  }

  if (strcmp(topic, sub_month) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Stage2_month = responseToFloat(payload, len);
    WebSerial.println("Stage 2 Month set to: " + String(N_rtc.N_month));
  }

  bool update_default_parameters = false;

  //F1 stg1 on/off time
  if (strcmp(topic, sub_f1_st1_ontime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    stage1_params.fanOnTime = responseToFloat(payload, len);
    WebSerial.println("F1 Stage 1 on time set to: " + String(stage1_params.fanOnTime) + " MINS");
    
    update_default_parameters = true;
  }

  if (strcmp(topic, sub_f1_st1_offtime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    stage1_params.fanOffTime = responseToFloat(payload, len);
    WebSerial.println("F1 Stage 1 off time set to: " + String(stage1_params.fanOffTime) + " MINS");
        
    update_default_parameters = true;
  }

  // F1 and S1 STAGE 2 on/off time
  if (strcmp(topic, sub_f1_st2_ontime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    stage2_params.fanOnTime = responseToFloat(payload, len);
    WebSerial.println("F1 Stage 2 on time set to: " + String(stage2_params.fanOnTime) + " MINS");
        
    update_default_parameters = true;
  }

  if (strcmp(topic, sub_f1_st2_offtime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    stage2_params.fanOffTime = responseToFloat(payload, len);
    WebSerial.println("F1 Stage 2 off time set to: " + String(stage2_params.fanOffTime) + " MINS");
        
    update_default_parameters = true;
  }

  if (strcmp(topic, sub_s1_st2_ontime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    stage2_params.sprinklerOnTime = responseToFloat(payload, len);
    WebSerial.println("S1 Stage 2 on time set to: " + String(stage2_params.sprinklerOnTime) + " MINS");
        
    update_default_parameters = true;
  }

  if (strcmp(topic, sub_s1_st2_offtime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    stage2_params.sprinklerOffTime = responseToFloat(payload, len);
    WebSerial.println("S1 Stage 2 off time set to: " + String(stage2_params.sprinklerOffTime) + " MINS");
        
    update_default_parameters = true;
  }

  // F1 and S1 STAGE 3 on/off time
  if (strcmp(topic, sub_f1_st3_ontime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    stage3_params.fanOnTime = responseToFloat(payload, len);
    WebSerial.println("F1 Stage 3 on time set to: " + String(stage3_params.fanOnTime) + " MINS");
        
    update_default_parameters = true;
  }

  if (strcmp(topic, sub_f1_st3_offtime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    stage3_params.fanOffTime = responseToFloat(payload, len);
    WebSerial.println("F1 Stage 3 off time set to: " + String(stage3_params.fanOffTime) + " MINS");
        
    update_default_parameters = true;
  }

  if (strcmp(topic, sub_s1_st3_ontime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    stage3_params.sprinklerOnTime = responseToFloat(payload, len);
    WebSerial.println("S1 Stage 3 on time set to: " + String(stage3_params.sprinklerOnTime) + " MINS");
        
    update_default_parameters = true;
  }

  if (strcmp(topic, sub_s1_st3_offtime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    stage3_params.sprinklerOffTime = responseToFloat(payload, len);
    WebSerial.println("S1 Stage 3 off time set to: " + String(stage3_params.sprinklerOffTime) + " MINS");
        
    update_default_parameters = true;
  }

  // Sub A and Sub B value update
  if (strcmp(topic, sub_A) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    room.A = responseToFloat(payload, len);
    // room.A = atoi((char *)payload);
    WebSerial.println("A set to: " + String(room.A));
        
    update_default_parameters = true;
  }

  if (strcmp(topic, sub_B) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    room.B = responseToFloat(payload, len);
    WebSerial.println("B set to: " + String(room.B));
        
    update_default_parameters = true;
  }

  // PID update
  if (strcmp(topic, sub_P) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Kp = responseToFloat(payload, len);
    WebSerial.println("P set to: " + String(Kp));
    R_P = 1;
  }

  if (strcmp(topic, sub_I) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Ki = responseToFloat(payload, len);
    WebSerial.println("I set to: " + String(Ki));
    R_I = 1;
  }

  if (strcmp(topic, sub_D) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Kd = responseToFloat(payload, len);
    WebSerial.println("D set to: " + String(Kd));
    R_D = 1;
  }

  if (R_P == 1 && R_I == 1 && R_D == 1 && START1 == 0 && START2 == 0 && STOP == 0) {
    air_in_feed_PID.SetTunings(Kp, Ki, Kd);
    WebSerial.println("New PID parameter updated");
    R_P = R_I = R_D = 0;
  }

  if (strcmp(topic, sub_coefPID) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    coefPID = responseToInt(payload, len);
    WebSerial.print("coef PID : " + String(coefPID));
  }

  // Target temperature Ts & Tc update
  if (strcmp(topic, sub_ts_set) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    temp_set.ts = responseToFloat(payload, len);
    WebSerial.println("Ts Condition set to: " + String(temp_set.ts));
        
    update_default_parameters = true;
  }

  if (strcmp(topic, sub_tc_set) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    temp_set.tc = responseToFloat(payload, len);
    // Tc_cond = temp_set->tc;
    WebSerial.println("Tc Condition set to: " + String(temp_set.tc));
        
    update_default_parameters = true;
  }

  // START
  if (strcmp(topic, sub_start) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_start = responseToInt(payload, len);
    WebSerial.println("START BUTTON PRESSED ON NODE RED" + String(N_start));
  }

  // D_START
  if ((strcmp(topic, sub_d_start) == 0) && START2 == 0 && STOP == 0) {
    N_d_start = responseToInt(payload, len);
    WebSerial.println("d_start BUTTON PRESSED ON NODE RED" + String(N_d_start));
  }

  // STOP
  if (strcmp(topic, sub_stop) == 0) {
    N_stop = responseToInt(payload, len);
    WebSerial.println("stop BUTTON PRESSED ON NODE RED" + String(N_stop));
  }

  // // Choose TS
  // if (strcmp(topic, sub_chooseTs) == 0) {
  //   N_chooseTs = responseToInt(payload, len);
  //   WebSerial.println("Ts is now IR" + String(N_chooseTs));

  //   // controller.set
  // }

  if(update_default_parameters) controller.updateDefaultParameters(stage1_params, stage2_params, stage3_params, room, temp_set);
}

//// Stop button pressed ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void stopRoutine() {
  if (stop_temp1 == 0) {
    WebSerial.println("PROCESS STOP INITIATED");
    controller.writeDigitalOutput(STAGE_1_IO, LOW);
    controller.writeDigitalOutput(STAGE_2_IO, LOW);
    controller.writeDigitalOutput(STAGE_3_IO, LOW);
    controller.writeDigitalOutput(VALVE_IO, LOW);
    controller.writeDigitalOutput(FAN_IO, LOW);
    controller.writeDigitalOutput(FAN2_IO, LOW);
    controller.writeAnalogOutput(AIR_PWM, LOW); // with chicano 

    Output = 0;
    coefOutput = 0; // inverted for chicano so 255=0V
    stop_temp1 = 1;

    F1_data.M_F1 = S1_data.M_S1 = 2;
    F2_data.M_F2 = 0;

    mqtt.publishData(m_F1, F1_data.M_F1);
    mqtt.publishData(m_F2, F2_data.M_F2);
    mqtt.publishData(m_S1, S1_data.M_S1);

    setStage(IDLE);

    WebSerial.println("Stage 0 Status Send packet ");
  }

  if (stop_temp2 == 0) {
    MTR_State = MTR2_State = C2_state = C3_state = S1_state = START1 = START2 = Stage2_started = Stage3_started = Stage2_RTC_set = 0;
    stop_temp2 = 1;
  }

  if (stop_temp2 == 1) {
    WebSerial.println("PROCESS STOPPED");
    stop_temp1 = stop_temp2 = STOP = 0;
  }
}

void updateTemperature() {
  TA = controller.readTempFrom(TA_AI);
  TS = controller.readTempFrom(TS_AI);
  TC = controller.readTempFrom(TC_AI);
  // TI = controller.readTempFrom(TI_AI);  // <=========== this shit it's da problem 

  // TA = 0;
  // TS = 0; // was desactivated
  // TC = 0;
}

// THIS SHOULD BE ALSO IN THE CONTROLLER

String addressToString(uint8_t *address) {
  String formated_address;
  for (int i = 0; i < 8; i++) {
    formated_address += address[i];
    if (i < 7) formated_address += ",";
  }
  return formated_address;
}

void setStage(SystemState Stage) {
  // if (stage_data.stage == Stage) return;
  currentState.stage = Stage;
  currentState.step = 0;
  mqtt.publishData(STAGE, Stage);

  controller.saveLastState(currentState);
}

float responseToFloat(byte *value, size_t len) {
  String puta_mierda_mal_parida;
  for (int i = 0; i < len; i++) puta_mierda_mal_parida += (char)value[i];
  return puta_mierda_mal_parida.toFloat();
}

int responseToInt(byte *value, size_t len) {
  String puta_mierda_mal_parida;
  for (int i = 0; i < len; i++) puta_mierda_mal_parida += (char)value[i];
  return puta_mierda_mal_parida.toInt();
}

void asyncLoopSprinkler(uint32_t &timer, uint32_t offTime, uint32_t onTime) {
  bool isSprinklerOn = controller.readDigitalInput(VALVE_IO);

  if (!isSprinklerOn && hasIntervalPassed(timer, offTime, true)) {
    // Es tiempo de encender el aspersor
    controller.writeDigitalOutput(VALVE_IO, HIGH);
    WebSerial.println("Sprinkler ON at: " + String(millis()));
    
    // publishStateChange(m_S1, true, "Stage 2 S1 Start published "); 
    mqtt.publishData(m_S1, S1_data.M_S1);
  } 
  
  else if (isSprinklerOn && hasIntervalPassed(timer, onTime, true)) {
    // Es tiempo de apagar el aspersor
    controller.writeDigitalOutput(VALVE_IO, LOW);
    WebSerial.println("Sprinkler OFF at: " + String(millis()));

    // publishStateChange(m_S1, false, "Stage 2 S1 Stop published ");
    mqtt.publishData(m_S1, S1_data.M_S1);
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