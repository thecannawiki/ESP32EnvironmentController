/**********
Hardware support and default pins
  * 1 PWM fan          pin12
  * 1 dehumidifier     pin14
  * 1 humidifier       pin13
  * 1 heater           pin33
  * 1 magnetic stirrer pin26
  * 1 neopixel         pin19
  I2C connections
  SDA                  pin21 
  SCL                  pin22

*********/
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "sdkconfig.h"
#include <Arduino.h>
#include <globals.h>
#include <WiFiManager.h>
#include <homepage.h>
#include <mathsfunctions.h>
#include <led.h>
#include <buffer.h>
#include <mqttResponse.h>
#include <NVM.h>
#include <peripherals.h>
#include <string>
#include <sensors.h>
#include <setup_page.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

extern "C" {
  #include "driver/touch_pad.h"
}

int sensorReadFailCount = 0;
int mqttPublishFailCount = 0;
int wifiConnectFailCount = 0;
int consecutiveMqttConnectFailCount =0;

int PIDLookback = 12; //number of samples to look back on for PID 

bool restoreDehumid = false;
bool restoreHumid = false;

char mqtturl[80] = "";
char mqttUsername[40] = "";
char mqttPass[40] = "";
char mqttPort[6] = "";

String MQTTURL = "";
String MQTTUSER = "";
String MQTTPASS = "";
String MQTTPORT = "";

// WiFiManager parameter objects
WiFiManagerParameter custom_deviceName("deviceName", "Device Name", deviceName, 40);
WiFiManagerParameter custom_mqtturl("mqttURL", "MQTT URL", mqtturl, 80);
WiFiManagerParameter custom_mqttuser("mqttuser", "MQTT username", mqttUsername, 40);
WiFiManagerParameter custom_mqttpass("mqttpass", "MQTT pass", mqttPass, 40);
WiFiManagerParameter custom_mqttport("mqttport", "MQTT port", mqttPort, 6);

// Base case: one argument left
template<typename T>
void logln(T last) {
    Serial.println(last);
}

// Recursive case: print one arg, then recurse
template<typename T, typename... Args>
void logln(T first, Args... rest) {
    Serial.print(first);
    Serial.print(' ');
    logln(rest...);
}


void saveWifiManagerCustomCredentials(){
  //TODO trim these user inputs!!
  const char* deviceStr = custom_deviceName.getValue();
  if(strlen(deviceStr) > 0){
    strcpy(deviceName, deviceStr);
    preferences.putString("deviceName", deviceName);
  }
  
  const char* url = custom_mqtturl.getValue();
  if(strlen(url) > 0){
    strcpy(mqtturl, url);
    preferences.putString("mqttUrl", mqtturl);
  }

  const char* pass = custom_mqttpass.getValue();
  if(strlen(pass) > 0){
    strcpy(mqttPass, pass);
    preferences.putString("mqttPass", mqttPass);
  }

  const char* username = custom_mqttuser.getValue();
  if(strlen(username) > 0){
    strcpy(mqttUsername, username);
    preferences.putString("mqttUser", mqttUsername);
  }

  const char* port = custom_mqttport.getValue();
  if(strlen(port) > 0){
    strcpy(mqttPort, port);
    preferences.putString("mqttPort", mqttPort);
  }

}

bool refreshNetworkTime(){
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return false;
  }
  return true;
}


void UpdateSensorJson(){
  StaticJsonDocument<420> doc;  
  char trimmedfloat[10];
  
  if(temp != 0.0f && tempBuffer.avgOfLastN(10) != 0.0f){ // First read on the scd40 can be 0
    dtostrf(temp, 5, 3, trimmedfloat);
    doc["T"] = atof(trimmedfloat);

    dtostrf(humidity, 5, 3, trimmedfloat);
    doc["RH"] = atof(trimmedfloat);
  }
  
  if(co2 != 65535){doc["co2"] = co2;}   // 65535 is the co2 value from the SCD-40 when its still warming up

  // doc["tvoc"] = tvoc;
  dtostrf(calcVpd(temp, humidity), 4, 2, trimmedfloat); 
  doc["vpd"] = atof(trimmedfloat);
  dtostrf(targetVpd, 4, 2, trimmedfloat); 
  doc["TVpd"] = atof(trimmedfloat);
  doc["AF"] = int(automaticFanVpd); //auto fan
  dtostrf(targetHumidity, 5, 2, trimmedfloat);
  doc["TRH"] = atof(trimmedfloat);        //Target RH (humidity)
  dtostrf(P, 4, 2, trimmedfloat);
  doc["P"] = atof(trimmedfloat);
  dtostrf(I, 4, 2, trimmedfloat);
  doc["I"] = atof(trimmedfloat);
  dtostrf(D, 4, 2, trimmedfloat);
  doc["D"] = atof(trimmedfloat);
  if(!lockHVAC){
    dtostrf(fanPower, 4, 1, trimmedfloat);
    doc["F"] = atof(trimmedfloat);  //fan
    doc["DHU"] = int(dehumidiferState);    //dehumidifier state
    doc["HU"] = int(humidifierState);
  }
  doc["ADHU"] = int(automaticDehumidifier);  //auto dehumidifier
  doc["AHU"] = int(automaticHumidifier);  //auto dehumidifier
  doc["w1"] = waterSensor1State;
  doc["w2"] = waterSensor2State;
  doc["H"] = int(heaterState); //heater
  doc["w1T"] = w1maxWaterSensorVal;
  doc["w2T"] = w2maxWaterSensorVal;
  doc["VPDMode"] = int(vpdMode);
  doc["ram"] = ESP.getFreeHeap();
  doc["LFB"] = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
  Serial.printf("Largest free block: %u bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));


  serializeJson(doc, sensorjson);
}

void initNonVolitileMem(){
  // TODO default values are duplicated here
  preferences.begin("controller", false); 
  fanPower = preferences.getFloat("fanPower", 0.0f);
  dehumidiferState = preferences.getBool("dehumidifier", dehumidiferState);
  autoHeater = preferences.getBool("autoHeater", autoHeater);
  P = preferences.getFloat("P", 0.4f);
  I = preferences.getFloat("I", 0.3f);
  D = preferences.getFloat("D", 3.0f);
  automaticDehumidifier = preferences.getBool("autoDehumid", automaticDehumidifier);
  automaticFanVpd = preferences.getBool("autoVpd", automaticFanVpd);
  dehumidifierPrimaryMode = preferences.getBool("primaryHumid", dehumidifierPrimaryMode);
  dehumidifierForTemp = preferences.getBool("dehumidAutoTemp", dehumidifierForTemp);
  targetVpd = preferences.getFloat("tvpd", 1.0f);
  targetTemperature = preferences.getFloat("ttemp", targetTemperature);
  targetHumidity = preferences.getFloat("targetHumidity", targetHumidity);
  heaterState = preferences.getBool("headerState", false);
  softMaxFan = preferences.getFloat("softMaxFan", 100.0f);
  softMinFan = preferences.getFloat("softMinFan", 0.0f);
  heaterTempMode = preferences.getFloat("HFT", heaterTempMode);
  humidifierState = preferences.getBool("humidifier", false);
  automaticHumidifier= preferences.getBool("AHU", true);
  vpdMode = preferences.getBool("VpdMode", true);
  w1maxWaterSensorVal = preferences.getInt("w1Max", 1800);
  w2maxWaterSensorVal = preferences.getInt("w2Max", 1800);
  setupMode = preferences.getBool("setupMode", false);
  strncpy(deviceName, preferences.getString("deviceName", "").c_str(), sizeof(deviceName));
  deviceName[sizeof(deviceName) - 1] = '\0';  // ensure null-terminated 
  MQTTURL = preferences.getString("mqttUrl", "");
  MQTTUSER = preferences.getString("mqttUser", "");
  MQTTPASS = preferences.getString("mqttPass", "");
  MQTTPORT = preferences.getString("mqttPort", "");
  
  UpdateSensorJson();
  Serial.println("retrieved from NVM: ");
  Serial.println(sensorjson);
}



//Handlers for web requests
// void getSensorReadingsWeb(AsyncWebServerRequest *request){
//   UpdateSensorJson();
//   request->send_P(200, "application/json", sensorjson);
// }


// void setUpperbound(AsyncWebServerRequest *request){
//   int paramsNr = request->params();
//   for(int i=0;i<paramsNr;i++){
//     const AsyncWebParameter* p = request->getParam(i);
//     String paramName = p->name();
//     String paramValue = p->value();
//     if(paramName=="int" && paramValue.toInt()!=0){
//       Serial.println("setting upper bound to");
//       Serial.println(paramValue);
//       upperHumidityBound = paramValue.toFloat();
//       request->send(200, "text/plain", "upperBound set");
//     } else {
//       Serial.println("cast failed or int param not found");
//       request->send(400, "cast failed or int param not found");
//     }
//   }
// }

// void setLowerbound(AsyncWebServerRequest *request){
//   int paramsNr = request->params();
//   for(int i=0;i<paramsNr;i++){
//     const AsyncWebParameter* p = request->getParam(i);
//     String paramName = p->name();
//     String paramValue = p->value();
//     if(paramName=="int" && paramValue.toInt()!=0){
//       Serial.println("setting lower bound to");
//       Serial.println(paramValue);
//       lowerHumidityBound = paramValue.toFloat();
//       Serial.println(lowerHumidityBound);
//       request->send(200, "text/plain", "lowerBound set");
//     } else {
//       Serial.println("cast failed or int param not found");
//       request->send(400, "cast failed or int param not found");
//     }
//   }
// }



void longPWMloop(void *parameter){ // 0-1000
  int *power;
  power = (int *) parameter;
  int totalwidth = 50;
  int highcount = totalwidth * int(power) / 1000; 
  //int halfPower = (softMaxPWM - 1)/ 2 + 1; //sweet trick to divide and get an int
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

void kickFan(){
  ledcWrite(fanPWMchannel, maxPWMval);
  delay(160);
  ledcWrite(fanPWMchannel, (0.1 * maxPWMval));
}

void updateFanPower(){ /// 0-100 to 1 dp. Value then gets converted to int 0-1000 by multiplying by 10

  int powerVal  = fanPower * 10;    // 0-1000

  if(powerVal>1000){powerVal = 1000;}
  if(powerVal<0){powerVal = 0;}

  if(powerVal == 0){
    ledcWrite(fanPWMchannel, 0);
  }
  powerVal = map(powerVal, 0, 1000, 0, fanSoftMaxPWM);


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

      ledcWrite(fanPWMchannel, map(val, 0, 1000, 0, fanSoftMaxPWM));
      vTaskDelay(300);
    }
  } 

  Serial.print("prev fan value ");
  Serial.print(fanBuffer.avgOfLastN(1));
  Serial.print(" ");
  if(fanBuffer.avgOfLastN(1) <= 1 && fanPower > 1){
    kickFan();
  }

  ledcWrite(fanPWMchannel, map(powerVal, 0, 1000, 0, fanSoftMaxPWM));

  preferences.putFloat("fanPower", fanPower);
  saveToNVM();
  Serial.println("fan updated");
  lastPowerVal = powerVal;
  fanBuffer.write(fanPower);
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
      Serial.print("WD ");
    }
  }
}

void reconnectWifi(){
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
}


void mqttreconnect() {    //TODO check mqtt stuff is defined
  if(WiFi.status() != WL_CONNECTED){
    logln("can't reconnect mqtt. No Wifi connection");
    return;
  }
  int errcount = 0;
  // Loop until we're reconnected.. If we loop forever here the watchdog will restart
  while (!mqttclient.connected()) {
    espClient.stop();
    Serial.println("Attempting MQTT connection...");
    
    bool connectSuccess = mqttclient.connect(WiFi.macAddress().c_str(), MQTTUSER.c_str(), MQTTPASS.c_str());
    if (connectSuccess) {
      consecutiveMqttConnectFailCount = 0;
      Serial.println("connected");
      // Subscribe
      std::string controlTopic;
      controlTopic = MQTTCONTROLTOPIC;
      mqttSubscribeTopics(controlTopic);
    }
    else {
        int errstate = mqttclient.state();
        logln("failed, mqtt client state=", errstate,"will try again");
        if(errstate==5){
            logln("Check credentials");
        }
        logln("mqttconnect", "mqtt url:",MQTTURL,"mqtt username:", MQTTUSER,"pass:",MQTTPASS, "mqtt port:",MQTTPORT);
        errcount++;
        consecutiveMqttConnectFailCount ++;
    }

    if(consecutiveMqttConnectFailCount> 10){
      logln("Too many fails getting mqtt connection.. restarting");
      ESP.restart();
    }
    
    if(errcount>1){
      break;
    }
    vTaskDelay(100);
  }
}


void mqttPublishSensorData(){ //TODO check if mqtt in creds
  UpdateSensorJson();
  bool success = mqttclient.publish(MQTTPUBLISHTOPIC, sensorjson, false);
  if(success){
    mqttPublishFailCount = 0;
    Serial.print("-> ");
  } else {
    mqttPublishFailCount +=1;
    Serial.print("MQTT send failed with"); Serial.println(sensorjson);
    if(mqttPublishFailCount > 5){
      Serial.println("mqtt publish failed 5 times. Restarting");
      ESP.restart();
    }
  }
}

void mqttMessageReceived(char* topic, byte* message, unsigned int length) {
  //esp_task_wdt_reset();
  String messageTemp;
  Serial.println("");
  
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }

  logln("got mqtt ", topic, " | ", messageTemp);
  mqttHandle(topic, messageTemp);
}

void operateDehumidifierOnBounds(){   //if humidity too high turn on the dehumidifier and vice-versa
  if(humidity>=(float)-1){
    Serial.print("O ");
    if(dehumidiferState){ //dehumidifier on
        if(humidity <= lowerHumidityBound){
          setDehumidifierState(false);
        }
    } else{ //dehumidifer off
      if(humidity >= upperHumidityBound){
        setDehumidifierState(true);
      }
    }
  }
}

void operateDehumidifierOnFanUsage(){
  if(!dehumidiferState){
    if (fanPower >= softMaxFan * 0.5f){ // 50% of max fan allowed 
      setDehumidifierState(true);
    }
  } else {
    if (fanPower <= softMaxFan * 0.2f){ // 20% of max fan allowed 
        setDehumidifierState(false);
    }
  }
}

void operateHumidifier(){
  float humidityError = humidity - targetHumidity;
  logln("hu");
  if(humidityError <= -0.5f && humidifierState == false){
    setHumidifierState(true);
    return;
  }
  if(humidityError >=0.0f && humidifierState == true){
    setHumidifierState(false);
  }
}

void operateDehumidifierOnTempBounds(){
 if(temp>=(float)-1){
    if(temp <= 24){
      setDehumidifierState(true);
    }
    if(temp >= 26){
      setDehumidifierState(false);
    }
 }
}

float calcDfan(){
  int lookback_length = 8;    //good for fast oscillations 

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
  if(diff > 0){
    Serial.print("RH trending up");
  } else {
    Serial.print("RH trending down");
  }
  Serial.println(")");

  return Dfan;
}

float calcIfan(){
  float sum = 0;
  int lookback = 24;

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
      // errorBufferWrite(humidityError);
      errorBuffer.write(humidityError);
      Serial.println("");
      Serial.print("humidity error ");
      Serial.print(humidityError,4);
      Serial.println("");
      Serial.println("PID ");
      //P
      float Pfan = P * humidityError;
      
      logln("P = ", Pfan);
      
      //D
      float Dfan = calcDfan();
      //I
      float Ifan = calcIfan();

      float newfan = fanPower + Pfan + Dfan + Ifan;
      if(newfan > softMaxFan){ newfan = softMaxFan;}
      if(newfan < softMinFan){newfan = softMinFan;}
      if(newfan <=0){newfan = 0.0f;}
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
      logln("Venting due to high temp! (Over", ventTemp, "degrees)");
      fanPower=softMaxFan;
      fanChanged=true;
    }
  }
} 

void operateHeater(){
  float humidityError = humidity - targetHumidity;  
  if(temp==-1 || tempBuffer.avgOfLastN(3) >= ventTemp //safety feature 👍
    || (tempBuffer.avgOfLastN(3) >= targetTemperature) ||  humidityError < 0.0f ){ //always care if we are push humidity too dry
    setHeaterState(false);
    return;
  } 

  if((!heaterTempMode && humidityError >= 2.5f) || (!heaterTempMode && fanPower >= 0.5f * softMaxFan) || (heaterTempMode && tempBuffer.avgOfLastN(5) < targetTemperature)){
    setHeaterState(true);
    if(fanPower > 15){
      fanPower = 15;
    }
    fanChanged = true;
  }
}

void operateHeaterOnBounds(){
  float humidityError = humidity - targetHumidity;  
  if(temp==-1 || temp >= 30){
    setHeaterState(false);
    return;
  } 
  
  if(humidityError <0){
    setHeaterState(false);
    return;
  }

  if(humidityError > 5){
    setHeaterState(true);
    return;
  }
}


void serialLog(const char str[], bool newLine){
  Serial.println("");
  Serial.print(timeinfo.tm_hour);
  Serial.print(":");
  Serial.print(timeinfo.tm_min);
  Serial.print(" - "); 

  if(newLine){
    Serial.println(str);
  } else {
    Serial.print(str);
  }

}

void send_water_added(time_t now, bool early_stop){
  float total_time = now - pumpStart;
  Serial.print("total run time ");
  Serial.println(total_time);

  char data[50];
  snprintf_P(data, sizeof(data), PSTR("{\"pump_time_active\":%f,\"early_stop\":%d}"), total_time, early_stop);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
}

void checkHeater(){
  if(heaterEnd == 0){
    setHeaterState(false);
  }
  time(&timeNow);

  if(timeNow >= heaterEnd){
    heaterEnd = 0;
    setHeaterState(false);
  }
}

void checkPump(){
  if(pumpEnd == 0){
    pumpState = false;
    digitalWrite(pumpControlPin, LOW);
    return;
  }
  
  time(&timeNow);

  if(waterSensor1State >= w1maxWaterSensorVal || waterSensor2State >= w2maxWaterSensorVal){
    pumpEnd = 0;
    pumpState = false;
    digitalWrite(pumpControlPin, LOW);
    Serial.print("Water detected! Stopping pump early ");
    send_water_added(timeNow, true);
    return;
  }

  if(timeNow >= pumpEnd){
    pumpEnd = 0;
    pumpState = false;
    digitalWrite(pumpControlPin, LOW);
    Serial.println("Pump timer ended");

    send_water_added(timeNow, false);
    pumpStart = 0;
  }
}


void updateWaterSensors(){
  w1Buffer.write(analogRead(waterSensor1Pin));
  // w1Buffer.printData();
  waterSensor1State = w1Buffer.avgOfLastN(8);
  // waterSensor2State = analogRead(waterSensor2Pin);

  serialLog("water sensor 1: ", false);
  Serial.println(waterSensor1State);

  w2Buffer.write(100 - touchRead(touchWaterSensor));
  waterSensor2State = w2Buffer.avgOfLastN(8);
  Serial.print("Touch vals: ");
  w2Buffer.printData();
  Serial.println("");
  logln(w2Buffer.avgOfLastN(1));
  Serial.println("");
}

void interactiveSetupTask(void * parameter){
  logln("Awaiting user setup");
  while(true){
    vTaskDelay(5000);
  }
}

void mainloop(void * parameter){
  vTaskDelay(1000);   //yield some time to other tasks
  int sensorTime = 5; //num of loop iterations between sensor reading/PID. Every x loops
  unsigned long interval = 1000;   // 1 second

    mqttreconnect();
  mqttclient.publish(MQTTPUBLISHTOPIC, "hi");

  while(true){
    unsigned long start = millis();      // when the loop started

    if(lockHVAC && dehumidiferState){   //turn off dehumidifer when HVAC is locked (this is now a sideffect of the variable state!)
      setDehumidifierState(false);
      restoreDehumid = true;
    }
    if(lockHVAC && humidifierState){
      setHumidifierState(false);
      restoreHumid = true;
    }
    if(lockHVAC && heaterState){
      setHeaterState(false);
    }

    // 0 here is for the transpiration test so fan power can change when Hvac is locked and power is 0 (at the end of the test)
    if(fanChanged && (!lockHVAC || fanPower == 0)){         
      updateFanPower();  //cool trick to recreate fan task at a good time (where it wont crash)
      fanChanged=false;
    }

    if(restoreDehumid && !lockHVAC){
      setDehumidifierState(true);
      restoreDehumid = false;
    }
    if(restoreHumid && !lockHVAC){
      setHumidifierState(true);
      restoreHumid = false;
    }
    
    if(loopCounter%5 == 0){
      updateWaterSensors();
      checkPump();
      // checkHeater();
      Serial.printf("Main Stack high watermark: %u\n", uxTaskGetStackHighWaterMark(NULL));
    }
    
    if(loopCounter%sensorTime == 0){
      bool gotSense = checkSensors();
      if(!gotSense){
        logln("No sensor found");
      } else {
        bool successful_read = getSensorReadings();
        if(successful_read){
          /// TODO check the value diffs. Throw out latest val if the diff is too big
          sensorReadFailCount = 0;
          if(vpdMode){
            targetHumidity = calcTargetHumidityForVpd(targetVpd, temp);
            preferences.putFloat("targetHumidity", targetHumidity);
            saveToNVM();
          } else {
            targetVpd = calcVpd(temp, targetHumidity);
            preferences.putFloat("tvpd", targetVpd);
            saveToNVM();
          }

          Serial.print("±RH ");
          Serial.print("fan ");
          Serial.println(fanPower);

          if(!lockHVAC){
            if(autoHeater){
              operateHeater(); 
            }

            if(automaticFanVpd){
              if(!heaterState){ // lock fan while heater is on
                fanPID();
              }
              ventOnHighTemp(); // allow vent in any case!
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
            if(automaticHumidifier){
              operateHumidifier();
            }
          }
        } else {   //sensor read failed
          sensorReadFailCount +=1;
          if(sensorReadFailCount >10){
            Serial.println("Sensor read failed >10 times in a row :/");

            Serial.println("Restarting esp");
            ///setting reasonable fan value
            setHeaterState(false);
            preferences.putFloat("fanPower", 40);
            saveToNVM();
            ESP.restart();

          }
        }
      }
    }

    if(loopCounter%10 == 0){  //Publish data
      if(WiFi.status() == WL_CONNECTED ){
        mqttreconnect();
        mqttPublishSensorData();
      } else {
        Serial.print("reconnecting wifi");
        bool success = WiFi.reconnect();
        if(success == ESP_OK){
          wifiConnectFailCount = 0;
          Serial.println("wifi reconnected");
        } else {
          Serial.println("wifi reconnect failed");
          wifiConnectFailCount++;
        }
        if(wifiConnectFailCount > 10){
          Serial.println("restarting due to failure to connect wifi");
          ESP.restart();
        }
      }
    }


    if(loopCounter %60 == 0){
      refreshNetworkTime();
      Serial.println("");
      Serial.print(timeinfo.tm_hour);
      Serial.print(":");
      Serial.print(timeinfo.tm_min);
      Serial.print(" ");
    }


    unsigned long elapsed = millis() - start;
    if (elapsed < interval) {
        vTaskDelay(interval - elapsed);
    }
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
      vTaskDelay(5000); // wait for main thread to reconnect us
    }
    vTaskDelay(800);
  }
}


void setup() {
  esp_log_level_set("mbedtls", ESP_LOG_VERBOSE);
  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  while (!Serial) { delay(300); } // Wait for serial console to open!
  delay(1000);
  // Wire.begin();
  Wire.begin(21, 22, 50000);  // 100 kHz
  //Wire.setClock(10000);

  pixels.begin();
  pixels.setBrightness(30);
  startLedanimation(rbgloop);

  //get NVM
  initNonVolitileMem();
  Serial.println("");
  delay(200);
  logln("Device name:", deviceName);
  heaterState = false;                  //safety 👍
  
  i2cScan();
  //sensors
  initSensors();
  delay(500);

  //Pin set up
  ledcSetup(fanPWMchannel, freq, resolution);
  ledcAttachPin(fanControlPin, fanPWMchannel);
  logln("Fan ledc setup");

  pinMode(pumpControlPin, OUTPUT);
  digitalWrite(pumpControlPin, LOW); 

  // pinMode(heaterControlPin, OUTPUT);
  // digitalWrite(heaterControlPin, LOW); 
  ledcSetup(heaterPWMchannel, heaterFreq, resolution);
  ledcAttachPin(heaterControlPin, heaterPWMchannel);
  logln("Heater ledc setup");

  pinMode(humidifierControlPin, OUTPUT);

  touch_pad_init();
  touch_pad_set_voltage(
    TOUCH_HVOLT_2V7,
    TOUCH_LVOLT_0V5,
    TOUCH_HVOLT_ATTEN_1V
  );
  touch_pad_config(TOUCH_PAD_NUM6, 0);

  // check if setup pin is high
  pinMode(setupModePin, INPUT_PULLUP);
  if (digitalRead(setupModePin) == LOW){
    setupMode = true;
  }

  WiFiManager wm;
  if(setupMode || deviceName[0] == '\0' || MQTTURL[0] == '\0' || MQTTPASS[0] == '\0' || MQTTUSER[0] == '\0' || MQTTPORT[0] == '\0'){
    if(LedAnimationTaskHandle != NULL){
      vTaskDelete(LedAnimationTaskHandle);
    }
    logln("In setup mode");
    setPixelColor(pixels.Color(224, 206, 0));  //yellow
    
    wm.addParameter(&custom_deviceName);
    wm.addParameter(&custom_mqtturl);
    wm.addParameter(&custom_mqttuser);
    wm.addParameter(&custom_mqttpass);
    wm.addParameter(&custom_mqttport);

    wm.startConfigPortal("canna");
    logln("portal Started");

    saveWifiManagerCustomCredentials();
    preferences.putBool("setupMode", false);
    saveToNVM();
    ESP.restart();
  }

  snprintf(MQTTPUBLISHTOPIC, sizeof(MQTTPUBLISHTOPIC), "%s/environ", deviceName);
  snprintf(MQTTCONTROLTOPIC, sizeof(MQTTCONTROLTOPIC), "%s/control", deviceName);
  logln("mqttpublishtopic", MQTTPUBLISHTOPIC );
  logln("mqttcontroltopic", MQTTCONTROLTOPIC );
  
  xTaskCreate(freezeWatchdog, "watchdog", 1000, 0, 0, &freezewatchdogTaskHandle);

  espClient.setInsecure();
  Serial.println("starting WIFI..");
  if(!wm.autoConnect()){  //if there is an available network, wait for connection
    int connectLoopCount = 0;
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(2000);
      connectLoopCount++;
      if(connectLoopCount > 20){
        break;
      }
    }

    if(WiFi.status() == WL_CONNECTED) {
      logln("Connected to: ", WiFi.SSID());
      Serial.println(WiFi.RSSI());
      logln("IP address: ", WiFi.localIP());
    } else {
      Serial.println("Wifi not connected");
    }
  }


  logln("MQTT URL ", MQTTURL);
  Serial.print("MQTT PORT ");
  Serial.println(MQTTPORT);
  startLedanimation(mqttconnectloop);
  mqttclient.setBufferSize(512);
  mqttclient.setServer(MQTTURL.c_str(), MQTTPORT.toInt());
  mqttclient.setCallback(mqttMessageReceived);

  vTaskDelete(LedAnimationTaskHandle);
  setPixelColor(pixels.Color(0,0,0));

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
  //TODO restore webserver functionality
  //http endpoints
  //   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
  //     request->send_P(200, "text/html", start_html);
  //   });
  //   server.on("/environ", getSensorReadingsWeb);
  //   server.begin();

  flash3green();

  // restore humidifer state
  digitalWrite(humidifierControlPin, humidifierState ? HIGH : LOW);
  updateFanPower();

  refreshNetworkTime();

}

extern "C" void app_main()
{
    // initialize arduino library before we start the tasks
    initArduino();
    setup();

    xTaskCreate(mqttLoop, "mqttHandler", 4000, 0, 1, &mqttTaskHandle);
    xTaskCreate(mainloop, "main", 24*1024, NULL, 5, NULL);
}
