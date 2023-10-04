#include <string>
#include <unordered_map>
#include <functional>
#include <Credentials.h>
#include <globals.h>
#include <mathsfunctions.h>
#include <NVM.h>
#include <peripherals.h>
std::unordered_map<std::string, std::function<void(const String&)>> functionDict;


// Function definitions (1/4)
const void handleFan(const String& message) {
    if(!lockHVAC){
      float num = message.toFloat();
      if(num>100){ num = 100;}
      if(num<0){num = 0;}
      if(num != fanPower){
        fanChanged = true;
      }
      fanPower = num;
      preferences.putFloat("fanPower", fanPower);
      saveToNVM();

      char data[20];
      snprintf_P(data, sizeof(data), PSTR("{\"fan\":%f}"), fanPower);
      mqttclient.publish(MQTTPUBLISHTOPIC, data);
    }
    else{
      Serial.println("oop HVAC controls locked");
    }
}

const void handleTranspirationMeasurement(const String& message){
  int time = message.toInt();
  if(time<1){
    mqttclient.publish(MQTTPUBLISHTOPIC, "time too short");
    return;
  }

  char data[60];
  float oldFanPower = fanPower;

  Serial.println("Starting transpiration test");
  lockHVAC = true;
  fanPower = 0;
  fanChanged = true;

  float startWater = calcGramsOfWaterInAir(temp, humidity);
  snprintf_P(data, sizeof(data), PSTR("{\"testRH1\":%f,\"water1\":%f}"), humidity, startWater);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);

  
  for (int i = 0; i < time; i++) {
    vTaskDelay(1000);  
  }
  lockHVAC = false;
  Serial.println("Test complete");
  fanPower = oldFanPower;
  fanChanged = true;

  float endWater = calcGramsOfWaterInAir(temp, humidity);
  snprintf_P(data, sizeof(data), PSTR("{\"waterDiff\":%f,\"water2\":%f}"), endWater - startWater, endWater);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
}

const void toggleDehumidifier(const String& message){
  if(!lockHVAC){
      setDehumidiferState(!dehumidiferState);
    }
}

const void handleTargetVpd(const String& message){
  float num = message.toFloat();
    if(num <= 2.0f && num >=0.4f){
      targetVpd = num;
      preferences.putFloat("tvpd", targetVpd);
      saveToNVM();
    }
}

const void handleSetP(const String& message){
  float num = message.toFloat();
  P = num;
  preferences.putFloat("P", P);
  saveToNVM();
}

const void handleSetI(const String& message){
  float num = message.toFloat();
  I = num;
  preferences.putFloat("I", I);
  saveToNVM();
}

const void handleSetD(const String& message){
  float num = message.toFloat();
  D = num;
  preferences.putFloat("D", D);
  saveToNVM();
}

const void handleSetDehumidifierAuto(const String& message){
  if(message == "on"){
      automaticDehumidifier = true;
      Serial.println("automatic dehumidifer control enabled");
    }
    else if(message == "off"){
      automaticDehumidifier = false;
      automaticVpd = false;
      Serial.println("automatic dehumidifer control disabled");
    }
    preferences.putBool("autoDehumid", automaticDehumidifier);
    saveToNVM();

    char data[50];
    snprintf_P(data, sizeof(data), PSTR("{\"autoDehumid\":%d}"), automaticDehumidifier);
    mqttclient.publish(MQTTPUBLISHTOPIC, data);
    //mqttPublishSensorData();
  }

const void handleSetDehumidiferAutoVpd(const String& message){
  if(message == "on"){
      automaticVpd = true;
    }
    else if(message == "off"){
      automaticVpd = false;
    }
    preferences.putBool("autoVpd", automaticVpd);
    saveToNVM();

    char data[50];
    snprintf_P(data, sizeof(data), PSTR("{\"autoVpd\":%d}"), automaticVpd);
    mqttclient.publish(MQTTPUBLISHTOPIC, data);
}

const void handlelowerBound(const String& message){
  float num = message.toFloat();
  lowerHumidityBound = num;
  preferences.putFloat("lowerBound", lowerHumidityBound);
  saveToNVM();
  char data[50];
  snprintf_P(data, sizeof(data), PSTR("{\"lowerBound\":%f}"), lowerHumidityBound);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
  //mqttPublishSensorData();
}

const void handleupperbound(const String& message){
  float num = message.toFloat();
  upperHumidityBound = num;
  preferences.putFloat("upperBound", upperHumidityBound);
  saveToNVM();
  char data[50];
  snprintf_P(data, sizeof(data), PSTR("{\"upperBound\":%f}"), upperHumidityBound);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
  //mqttPublishSensorData();
}

const void handlesetHeater(const String& message){
    if(message == "on"){
      heaterState = true;
      digitalWrite(heaterControlPin, HIGH); 
    }
    else if(message == "off"){
      heaterState = false;
      digitalWrite(heaterControlPin, LOW); 
    }
    preferences.putBool("headerState", heaterState);
    saveToNVM();
    char data[50];
    snprintf_P(data, sizeof(data), PSTR("{\"heaterState\":%d}"), heaterState);
    mqttclient.publish(MQTTPUBLISHTOPIC, data);
    //mqttPublishSensorData();
  }

/////

/////// define function key (2/4)
const std::string fanPowerTopicName = (MQTTCONTROLTOPIC + std::string{"/exhaust"}).data();
const std::string transpirationMeasurement = (MQTTCONTROLTOPIC + std::string{"/transTest"}).data();
const std::string dehumidiferToggle = (MQTTCONTROLTOPIC + std::string{"/dehumidifier/toggle"}).data();
const std::string dehumidiferAuto = (MQTTCONTROLTOPIC + std::string{"/dehumidifier/auto"}).data();
const std::string dehumidiferautoVpd = (MQTTCONTROLTOPIC + std::string{"/dehumidifier/autoVpd"}).data();
const std::string dehumidiferLower = (MQTTCONTROLTOPIC + std::string{"/dehumidifier/lower"}).data();
const std::string dehumidiferUpper = (MQTTCONTROLTOPIC + std::string{"/dehumidifier/upper"}).data();
const std::string setTargetVpd = (MQTTCONTROLTOPIC + std::string{"/targetVpd"}).data();
const std::string setP = (MQTTCONTROLTOPIC + std::string{"/P"}).data();
const std::string setI = (MQTTCONTROLTOPIC + std::string{"/I"}).data();
const std::string setD = (MQTTCONTROLTOPIC + std::string{"/D"}).data();
//////////



/// @brief  (3/4)
void mqttSubscribeTopics(){
  mqttclient.subscribe(dehumidiferAuto.data());
  mqttclient.subscribe(dehumidiferautoVpd.data());
  mqttclient.subscribe(dehumidiferLower.data());
  mqttclient.subscribe(dehumidiferUpper.data());
  mqttclient.subscribe(dehumidiferToggle.data());
  mqttclient.subscribe(fanPowerTopicName.data());
  mqttclient.subscribe(transpirationMeasurement.data());
  mqttclient.subscribe(setTargetVpd.data());
  mqttclient.subscribe(setP.data());
  mqttclient.subscribe(setI.data());
  mqttclient.subscribe(setD.data());
}


void mqttHandle(char* topic, String message) {
    ////Add to map here (4/4)
    functionDict[fanPowerTopicName] = handleFan;
    functionDict[transpirationMeasurement] = handleTranspirationMeasurement;
    functionDict[dehumidiferAuto] = handleSetDehumidifierAuto;
    functionDict[dehumidiferautoVpd] = handleSetDehumidiferAutoVpd;
    functionDict[dehumidiferLower] = handlelowerBound;
    functionDict[dehumidiferUpper] = handleupperbound;
    functionDict[dehumidiferToggle] = toggleDehumidifier;
    functionDict[setTargetVpd] = handleTargetVpd;
    functionDict[setP] = handleSetP;
    functionDict[setI] = handleSetI;
    functionDict[setD] = handleSetD;
    
    // Call the corresponding function based on the input
    std::string topicName = topic;
    auto it = functionDict.find(topicName);
    if (it != functionDict.end()) {
        // Function found in the dictionary, call it
        std::function<void(const String&)> selectedFunction = it->second;
        selectedFunction(message);
    } else {
        // Function not found
        Serial.println("Invalid function name!\n");
    }
}
