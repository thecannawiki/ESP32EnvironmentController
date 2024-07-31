/**********
Hardware support and default pins
  * 1 PWM fan          pin12
  * 1 dehumidifier     pin13
  * 1 heater           pin33
  * 1 magnetic stirrer pin26
  * 1 neopixel         pin19
  I2C connections
  SDA                  pin21 
  SCL                  pin22

*********/

#include <globals.h>
#include <homepage.h>
#include <mathsfunctions.h>
#include <led.h>
#include <buffer.h>
#include <mqttResponse.h>
#include <NVM.h>
#include <peripherals.h>
#ifdef BME
  #include <bme.h>
#endif
#ifdef SCD4
  #include <scd4.h>
#endif

int sensorMountFailCount = 0;
int sensorReadFailCount = 0;

int PIDLookback = 12; //number of samples to look back on for PID 

bool restoreDehumid = false;

//Functions for debug
void scanForWifi(){
  Serial.print("wifi Scan start ... ");
  int n = WiFi.scanNetworks();
  Serial.print(n);
  Serial.println(" network(s) found");
  for (int i = 0; i < n; i++)
  {
    Serial.println(WiFi.SSID(i));
    Serial.println(WiFi.RSSI(i));
  }
  Serial.println();
}

void i2cScan(){
  byte error, address; //variable for error and I2C address
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}


tm refreshNetworkTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return timeinfo;
  }
  
  return timeinfo;
}


void UpdateSensorJson(){
  StaticJsonDocument<320> doc;  
  char trimmedfloat[10];
  
  dtostrf(temp, 5, 3, trimmedfloat);  
  doc["T"] = trimmedfloat;
  
  dtostrf(humidity, 5, 4, trimmedfloat); 
  doc["RH"] = trimmedfloat;
  
  if(co2 != 65535){doc["co2"] = co2;}   // 65535 is the co2 value from the SCD-40 when its still warming up

  // doc["tvoc"] = tvoc;
  dtostrf(calcVpd(temp, humidity), 5, 4, trimmedfloat); 
  doc["vpd"] = trimmedfloat;
  dtostrf(targetVpd, 5, 3, trimmedfloat); 
  doc["targetVpd"] = trimmedfloat;
  doc["autoFanVpd"] = automaticFanVpd;
  dtostrf(targetHumidity, 5, 3, trimmedfloat);
  doc["targetHu"] = trimmedfloat;
  dtostrf(P, 5, 3, trimmedfloat);
  doc["P"] = trimmedfloat;
  dtostrf(I, 5, 3, trimmedfloat);
  doc["I"] = trimmedfloat;
  dtostrf(D, 5, 3, trimmedfloat);
  doc["D"] = trimmedfloat;
  if(!lockHVAC){
    dtostrf(fanPower, 5, 1, trimmedfloat);
    doc["fan"] = trimmedfloat;
    doc["dehumidState"] = dehumidiferState;
  }
  doc["autoDehumid"] = automaticDehumidifier;
  // too much data
  
  //doc["primaryHumid"] = dehumidifierPrimaryMode;

    // dtostrf(lowerHumidityBound, 5, 4, trimmedfloat);
  // doc["lowerBound"] = trimmedfloat;
  // dtostrf(upperHumidityBound, 5, 4, trimmedfloat);
  // doc["upperBound"] = trimmedfloat;
  doc["heater"] = heaterState;
  //doc["ram"] = ESP.getFreeHeap();
  serializeJson(doc, sensorjson);
}

void initNonVolitileMem(){
  // TODO default values are duplicated here
  preferences.begin("controller", false); 
  fanPower = preferences.getFloat("fanPower", 0.0f);
  dehumidiferState = preferences.getBool("dehumidifier", false);
  lowerHumidityBound = preferences.getFloat("lowerBound", 40.0f);
  upperHumidityBound = preferences.getFloat("upperBound", 60.0f);
  P = preferences.getFloat("P", 0.2f);
  I = preferences.getFloat("I", 0.2f);
  D = preferences.getFloat("D", 10.0f);
  automaticDehumidifier = preferences.getBool("autoDehumid", true);
  automaticFanVpd = preferences.getBool("autoVpd", true);
  dehumidifierPrimaryMode = preferences.getBool("primaryHumid", false);
  dehumidifierForTemp = preferences.getBool("dehumidAutoTemp", false);
  targetVpd = preferences.getFloat("tvpd", 1.0f);
  heaterState = preferences.getBool("headerState", false);
  softMaxFan = preferences.getFloat("softMaxFan", 100.0f);
  
  UpdateSensorJson();
  Serial.println("retrieved from NVM: ");
  Serial.println(sensorjson);
}



//Handlers for web requests
void getSensorReadingsWeb(AsyncWebServerRequest *request){
  UpdateSensorJson();
  request->send_P(200, "application/json", sensorjson);
}

void pressDehumidifierButtonWeb(AsyncWebServerRequest *request){
  setDehumidiferState(!dehumidiferState);
  request->send(200, "text/plain", "Dehumidifier switched");
}

void setUpperbound(AsyncWebServerRequest *request){
  int paramsNr = request->params();
  for(int i=0;i<paramsNr;i++){
    AsyncWebParameter* p = request->getParam(i);
    String paramName = p->name();
    String paramValue = p->value();
    if(paramName=="int" && paramValue.toInt()!=0){
      Serial.println("setting upper bound to");
      Serial.println(paramValue);
      upperHumidityBound = paramValue.toFloat();
      request->send(200, "text/plain", "upperBound set");
    } else {
      Serial.println("cast failed or int param not found");
      request->send(400, "cast failed or int param not found");
    }
  }
}

void setLowerbound(AsyncWebServerRequest *request){
  int paramsNr = request->params();
  for(int i=0;i<paramsNr;i++){
    AsyncWebParameter* p = request->getParam(i);
    String paramName = p->name();
    String paramValue = p->value();
    if(paramName=="int" && paramValue.toInt()!=0){
      Serial.println("setting lower bound to");
      Serial.println(paramValue);
      lowerHumidityBound = paramValue.toFloat();
      Serial.println(lowerHumidityBound);
      request->send(200, "text/plain", "lowerBound set");
    } else {
      Serial.println("cast failed or int param not found");
      request->send(400, "cast failed or int param not found");
    }
  }
}

void rampUpFan(int desiredPowerPercentage){ // int 0-100   //ramp fan up from the minimum power required to the desired power
  Serial.print("ramping up to");
  Serial.println(desiredPowerPercentage);
  float minPowerPercentage = minpercentvalue * softMaxPWM;
  if(desiredPowerPercentage<minpercentvalue){
    ledcWrite(fanPWMchannel, desiredPowerPercentage);
    return;
  }

  float power = map(desiredPowerPercentage, 0, 100, 0, softMaxPWM);

  for (int i = minPowerPercentage; i < power; i++) {
      //float power = map(i, 0, softMaxPWM, 0, 100);
      ledcWrite(fanPWMchannel, i);
      //Serial.println(power);
      vTaskDelay(5);
  }

}


void longPWMloop(void *parameter){ // 0-1000
  int *power;
  power = (int *) parameter;
  int totalwidth = 50;
  int highcount = totalwidth * int(power) / 1000; 
  int halfPower = (softMaxPWM - 1)/ 2 + 1; //sweet trick to divide and get an int
  int count = 0;
  while(true){ 
    if(count < highcount * 2){ //for halfpower change
      //rampUpFan(50);
      ledcWrite(fanPWMchannel, 20);
    } else {
      ledcWrite(fanPWMchannel, 0);
    }
    count+=1;
    if(count > totalwidth){
      count = 0;
    }
    if(fanChanged){  vTaskDelete( NULL ); Serial.println("fan task killed");}
    vTaskDelay(2000);
  }
}

void updateFanPower(){ /// 0-100 to 1 dp. Value then gets converted to int 0-1000 by multiplying by 10

  int powerVal  = fanPower * 10;    // 0-1000
  // float percentVal = powerVal / 1000.0f; //float 0 - 1

  if(powerVal>1000){powerVal = 1000;}
  if(powerVal<0){powerVal = 0;}

  if(powerVal == 0){
    ledcWrite(fanPWMchannel, 0);
  }
  powerVal = map(powerVal, 0, 1000, 0, softMaxPWM);


  // if(percentVal < minpercentvalue){
  //   Serial.println("long pwm task");
  //     ///let the pwm loop task handle the fan
  //   xTaskCreate(
  //   longPWMloop,      // Function that should be called
  //   "longPWM",        // Name of the task (for debugging)
  //   1000,             // Stack size (bytes)
  //   (void *) powerVal,// Parameter to pass
  //   3,                // Task priority
  //   &longPWMTaskHandle);       // Task handle 
  //   //Serial.print("made new fan task. fan value ");
  //   //Serial.println(powerVal);
  // }

  // Serial.print("Power val");
  // Serial.println(powerVal);

  int moveSize = 200;
  float fanDiff = powerVal - lastPowerVal;

  // Serial.print("diff");
  // Serial.println(fanDiff);

  if(fanDiff > moveSize){
    Serial.println("ramping up fan");
    Serial.print(fanDiff/moveSize);
    Serial.println(" steps");

    for(int i=1; i<(fanDiff/moveSize);i++){ //increase fan power in increments of moveSize
      int val = i*moveSize;
      Serial.print("fan ramp up ");
      Serial.println(val);

      ledcWrite(fanPWMchannel, map(val, 0, 1000, 0, softMaxPWM));
      vTaskDelay(300);
    }
  } 

  ledcWrite(fanPWMchannel, map(powerVal, 0, 1000, 0, softMaxPWM));

  preferences.putFloat("fanPower", fanPower);
  saveToNVM();
  Serial.println("fan updated");
  lastPowerVal = powerVal;
  //esp_task_wdt_reset();
}


void freezeWatchdog(void * parameter){
  int lastloopCounter=0;
  while(true){
    lastloopCounter = loopCounter;
    vTaskDelay(20000); //20 seconds
    if(loopCounter == lastloopCounter){
      Serial.println("Controller is frozen!!");
      ESP.restart();
    } else {
      Serial.print(" WD ");
    }
  }
}

void reconnectWifi(){
  try{
    wifiMulti.run();
    int waitCount = 0;
    Serial.println("");
    Serial.print("Connecting wifi");
    while (WiFi.status() != WL_CONNECTED){
      Serial.print(".");
      delay(2000);
      waitCount++;
      if(waitCount > 10){
        Serial.print("");
        break;
      }
    }
  } catch (std::exception& e) {
      Serial.println("wifi exception");
  }
}


void mqttreconnect() {    //TODO check mqtt stuff is defined
  if(WiFi.status() != WL_CONNECTED){
    Serial.println("can't reconnect mqtt. No Wifi connection");
    return;
  }
  int errcount = 0;
  // Loop until we're reconnected
  while (!mqttclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
   // String clientId = "esp32";
    bool connectSuccess = mqttclient.connect(WiFi.macAddress().c_str(), MQTTUSER, MQTTPASS);
    if (connectSuccess) { //TODO: move out of if and add error handling
      Serial.println("connected");
      // Subscribe
      mqttSubscribeTopics();
    } else {
      Serial.print("failed, client state=");
      Serial.print(mqttclient.state());
      Serial.println("will try again");
      errcount++;
      if(errcount>5){
        ESP.restart();
      }
    }
  }
}


void mqttPublishSensorData(){ //TODO check if mqtt in creds
  UpdateSensorJson();
  bool success = mqttclient.publish(MQTTPUBLISHTOPIC, sensorjson, false);
  if(success){
    Serial.print("-> ");
  } else {
    Serial.print("MQTT send failed with"); Serial.println(sensorjson);
  }
  
}

void mqttMessageReceived(char* topic, byte* message, unsigned int length) {
  //esp_task_wdt_reset();
  String messageTemp;
  Serial.println("");
  
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }

  Serial.print("got mqtt ");
  Serial.print(topic);
  Serial.print(" | ");
  Serial.println(messageTemp);

  mqttHandle(topic, messageTemp);
}

void operateDehumidifierOnBounds(){   //if humidity too high turn on the dehumidifier and vice-versa
  if(humidity>=(float)-1){
    Serial.print("O ");
    if(dehumidiferState){ //dehumidifier on
        if(humidity <= lowerHumidityBound){
          setDehumidiferState(false);
        }
    } else{ //dehumidifer off
      if(humidity >= upperHumidityBound){
        setDehumidiferState(true);
      }
    }
  }
}

void operateDehumidifierOnFanUsage(){
  if(!dehumidiferState){
    if (fanPower >= softMaxFan * 0.9f){ // 90% of max fan allowed 
      setDehumidiferState(true);
    }
  } else {
    if (fanPower <= softMaxFan * 0.7f){ // 50% of max fan allowed 
        setDehumidiferState(false);
    }
  }
}

void operateDehumidifierOnTempBounds(){
 if(temp>=(float)-1){
    if(temp <= 24){
      setDehumidiferState(true);
    }
    if(temp >= 26){
      setDehumidiferState(false);
    }
 }
}

float calcDfan(){
  // Serial.println("buffer");
  // for(int i=0; i<BUFFER_SIZE; i++){
  //   Serial.println(buffer.data[i]);
  // }
  int lookback_length = 7;    //good for fast oscillations 

  int lookback_index = humidityBuffer.newest_index-lookback_length;
  int cindex = humidityBuffer.newest_index-1;
  if(lookback_index < 0){
    lookback_index = BUFFER_SIZE + lookback_index;
  }
  if(cindex < 0){
    cindex = BUFFER_SIZE + cindex;
  }
  float old = humidityBuffer.data[lookback_index];
  float current = humidityBuffer.data[cindex];
  if(old ==-1.0f || old == 0.0f || current ==-1.0f || current== 0.0f){
    return 0;
  }

  float diff = ( current - old) / lookback_length;
  
  float Dfan = D * diff; 

  Serial.print("D = ");
  Serial.print(Dfan);
  Serial.print(" (diff ");
  Serial.print(diff);
  Serial.println(")");

  return Dfan;
}

float calcIfan(){
  float sum = 0;
  int lookback = 18;

  for(int i=1; i<lookback; i++){
    int cindex = errorBuffer.newest_index-i;
    if(cindex < 0){
      cindex = BUFFER_SIZE + cindex;
    }
    
    float c = errorBuffer.data[cindex];

    sum+=c;
  }

  float Ifan = (I * sum) /10; //for tuning
  
  Serial.print("I = ");
  Serial.print(Ifan);
  Serial.print(" (integral ");
  Serial.print(sum);
  Serial.println(")");

  return Ifan; 
}


void fanPID(){
      float humidityError = humidity - targetHumidity;
      errorBufferWrite(humidityError);
      Serial.println("");
      Serial.print("humidity error ");
      Serial.print(humidityError,4);
      Serial.println("");
      Serial.println("PID ");
      //P
      float Pfan = P * humidityError;
      
      Serial.print("P = ");
      Serial.print(Pfan);
      Serial.println("");
      
      //D
      float Dfan = calcDfan();
      //I
      float Ifan = calcIfan();

      float newfan = fanPower + Pfan + Dfan + Ifan;
      if(newfan > softMaxFan){ newfan = softMaxFan;}
      if(newfan <=0){newfan = 0.1f;}
      if(newfan >=100){newfan = 100;}
      if(abs(newfan - fanPower) > 0.0f){
        Serial.print("new fan power ");
        Serial.println(newfan);
        fanPower=newfan;
        fanChanged=true;
      }
}

void ventOnHighTemp(){  //safety feature 👍
  if(temp!=-1){
    if(temp > ventTemp){
      Serial.print("Venting due to high temp! (Over ");
      Serial.print(ventTemp);
      Serial.println(" degrees)");
      fanPower=100;
      fanChanged=true;
    }
  }
}

void operateHeaterOnBounds(){
  if(temp!=-1){
    if(temp>= targetTemperature +1){
      setHeaterState(false);
    }
    else if(temp<= targetTemperature -1 ){
      setHeaterState(true);
    }
  }
}


void checkSensors(){
  #ifdef SCD4
    if(!SCD40Mounted){
      bool ok = initSensors();
      if(!ok){
        sensorMountFailCount +=1;
      } else {
        sensorMountFailCount = 0;
      }
      return;
    }
  #endif

  #ifdef BME
    if(!bmeMounted){
      bool ok = initSensors();
      if(!ok){
        sensorMountFailCount +=1;
      } else {
        sensorMountFailCount = 0;
      }
      return;
    }
  #endif


  if(sensorMountFailCount >= 8){
    Serial.println("Failed to connect sensors >4 times in a row. SAFETY MODE");
    temp = -1;
    humidity = -1;
    fanPower = 30;
    fanChanged = true;
  }


  float num = 0;
  int dupecount = 0;
  for(int i=1; i<5; i++){
    int index = humidityBuffer.newest_index-i;
    if(index < 0){
        index = BUFFER_SIZE - index;
    }
    float c  = humidityBuffer.data[index];
    if(c == num){
        dupecount++;
    }
    num = c;
  }

  if(dupecount > 2){
    Serial.println("reinit sensors");
    initSensors();
  }
}

void send_water_added(time_t now){

  float total_time = now - pumpStart;
  Serial.print("total run time ");
  Serial.println(total_time);

  char data[32];
  snprintf_P(data, sizeof(data), PSTR("{\"pump_time_active\":%f}"), total_time);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
}

void checkPump(){
  if(pumpEnd == 0){
    pumpState = false;
    digitalWrite(pumpControlPin, LOW);
    return;
  }

  time_t now;
  time(&now);

  // if(waterSensor1State >0){
  //   pumpEnd = 0;

  //   pumpState = false;
  //   digitalWrite(pumpControlPin, LOW);
  //   Serial.print("stopping pump early ");
  //   send_water_added(now);
  //   return;
  // }

  
  // Serial.print("check epoch pump");
  // Serial.println(now);
  // Serial.print("pumpEnd time");
  // Serial.println(pumpEnd);
  if(now >= pumpEnd){
    pumpEnd = 0;
    pumpState = false;
    digitalWrite(pumpControlPin, LOW);
    Serial.println("Pump timer ended");

    send_water_added(now);
    pumpStart = 0;
  }
}



void updateWaterSensors(){
  waterSensor1State = analogRead(waterSensor1Pin);
  waterSensor2State = analogRead(waterSensor2Pin);
  Serial.println("");
  Serial.print("wtr snsr 1: ");
  Serial.print(waterSensor1State);
  Serial.print("   wtr snsr 2: ");
  Serial.println(waterSensor2State);
}


void mainloop(void * parameter){
  vTaskDelay(1000);   //yield some time to other threads
  int sensorTime = 10; //num of loop interations between sensor reading/PID. Every x loops
  #ifdef BME
    sensorTime = 5; //BME lets u sample every 5 seconds!
  #endif

  while(true){
    // 0 here is for the transpiration test so fan power can change when Hvac is locked and power is 0 (at the end of the test)
    if(fanChanged && (!lockHVAC || fanPower == 0)){         
      updateFanPower();  //cool trick to recreate fan task at a good time (where it wont crash)
      fanChanged=false;
    }

    if(restoreDehumid && !lockHVAC){
      setDehumidiferState(true);
      restoreDehumid = false;
    }

    if(lockHVAC && dehumidiferState){   //turn off dehumidifer when HVAC is locked (this is now a sideffect of the variable state!)
      setDehumidiferState(false);
      restoreDehumid = true;
    }


    if(loopCounter%3 == 0){
      updateWaterSensors();
      checkPump();
    }
    
    if(loopCounter%sensorTime == 0){
      checkSensors();
      bool successful_read = getSensorReadings();
      operateHeaterOnBounds();  //Should put this under "auto" switch

      if(successful_read){
        sensorReadFailCount = 0;
        targetHumidity = calcTargetHumidityForVpd(targetVpd, temp);
        Serial.print("±RH ");
        upperHumidityBound = targetHumidity + 0.5f;
        lowerHumidityBound = targetHumidity - 0.5f;

        if(!lockHVAC){
          if(automaticFanVpd){
            fanPID();
            ventOnHighTemp();
          }
          if(automaticDehumidifier){
            if(dehumidifierPrimaryMode){
                operateDehumidifierOnBounds();
            } else {
              // secondary mode
              if(dehumidifierForTemp) {
                operateDehumidifierOnTempBounds();
              } else {
                operateDehumidifierOnFanUsage();
              }
            }
          }
        }
      } else {
        sensorReadFailCount +=1;
        if(sensorReadFailCount >4){
          Serial.println("Sensor read failed >4 times in a row :/");
          if(!initSensors()){
            Serial.println("Reinit failed :/ Restarting esp");
            ///setting reasonable fan value
            preferences.putFloat("fanPower", 40);
            saveToNVM();
            ESP.restart();
          }
        }
      }
    }

    if(loopCounter%10 == 0){  //Publish data
      if(WiFi.status() == WL_CONNECTED && mqttclient.connected()){
        mqttPublishSensorData();
      }
    }

    if(loopCounter%20 == 0){
      Serial.print("checking wifi ..");
      if(WiFi.status() != WL_CONNECTED){
        bool success = WiFi.reconnect();
        if(success == ESP_OK){
          Serial.println("connected");
        }
        Serial.println("failed");
        // ESP.restart();
      }
    }

    if(loopCounter %60 == 0){
      tm time = refreshNetworkTime();
      Serial.println("");
      Serial.print(time.tm_hour);
      Serial.print(":");
      Serial.print(time.tm_min);
      Serial.println("");
    }

    vTaskDelay(1000);
    loopCounter++;    //changing this value resets the freezewatchdog
    if(loopCounter > 100){
      loopCounter = 0;
    }
  }
}

void mqttLoop(void * parameter){ //Keeps the mqtt client connected and receives/handles messages
  Serial.println("");
  Serial.println("mqtt client listening");
  while(true){
    if(mqttclient.connected()){
      Serial.print("¤ ");
      mqttclient.loop();
    } else {
      try{
        //Serial.println("");
        //Serial.print("ram b4 ssl ");
        //Serial.print(ESP.getFreeHeap());
        mqttreconnect();
        //Serial.print("ram after ssl ");
        //Serial.print(ESP.getFreeHeap());
        //Serial.println("");
        //mqttclient.loop();
      } catch (std::exception& e) {
        Serial.print("mqtt reconnect failed");
        vTaskDelay(5000);
      }
    }
    vTaskDelay(2000);
  }
}




void setup() {
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector 
  Serial.begin(115200);
  while (!Serial) { delay(300); } // Wait for serial console to open!
  delay(1000);
  Wire.begin();
  //Wire.setClock(10000);
  xTaskCreate(freezeWatchdog, "watchdog", 1000, 0, 0, &freezewatchdogTaskHandle);

  if(LED){
    pixels.begin();
    pixels.setBrightness(30);
    startLedanimation(rbgloop);
  }

  //get NVM
  initNonVolitileMem();
  Serial.println("");
  delay(200);

  //Pin set up
  ledcSetup(fanPWMchannel, freq, resolution);
  ledcAttachPin(fanControlPin, fanPWMchannel);
  pinMode(pumpControlPin, OUTPUT);
  digitalWrite(pumpControlPin, LOW); 
  pinMode(heaterControlPin, OUTPUT);
  digitalWrite(heaterControlPin, LOW); 
  //water sensors
  pinMode(waterSensor1Pin, INPUT);
  pinMode(waterSensor2Pin, INPUT_PULLUP);
  heaterState = false;                  //safety 👍
  
  //sensors
  initSensors();
  delay(1000);

  //Connect wifi
  Serial.println("starting WIFI..");
  wifiMulti.addAP(SSIDNAME, WIFIPASS);    //Add additional networks here
  uint8_t isWifi = wifiMulti.run();
  Serial.println(isWifi);
  Serial.println(WiFi.status());

  if(isWifi != 255){  //if there is an available network, wait for connection
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(2000);
      //TODO bail out after a timeout
    }

    if(WiFi.status() == WL_CONNECTED) {
      Serial.print("Connected to: ");
      Serial.println(SSIDNAME);
      Serial.println(WiFi.RSSI());
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    }
  }

  //Setup MQTT TODO check MQTT info is defined in creds
  Serial.print("MQTT URL ");
  Serial.println(MQTTURL);
  Serial.print("MQTT PORT ");
  Serial.println(MQTTPORT);
  startLedanimation(mqttconnectloop);
  mqttclient.setServer(MQTTURL, MQTTPORT);
  mqttclient.setCallback(mqttMessageReceived);
  mqttreconnect();
  mqttclient.publish(MQTTPUBLISHTOPIC, "hi");
  vTaskDelete(LedAnimationTaskHandle);
  setPixelColor(pixels.Color(0,0,0));

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
 
  //http endpoints
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", start_html);
  });
  server.on("/highBound", setUpperbound);
  server.on("/lowBound", setLowerbound);  
  server.on("/environ", getSensorReadingsWeb);
  server.on("/press", pressDehumidifierButtonWeb);

  server.begin();
  flash3green();
  xTaskCreate(mqttLoop, "mqttHandler", 4000, 0, 1, &mqttTaskHandle);
  xTaskCreate(mainloop, "main", 90000, 0, 0, &mainLoopTaskHandle);
  updateFanPower();

}


//not used
void loop(){}