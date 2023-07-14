#include <string>
#include <unordered_map>
#include <functional>
#include <Credentials.h>
#include <globals.h>
#include <NVM.h>
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
  char data[25];

  snprintf_P(data, sizeof(data), PSTR("{\"testRH\":%f}"), humidity);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);

  Serial.println("Starting transpiration test");
  lockHVAC = true;
  fanPower = 0;
  int time = 20;
  
  for (int i = 0; i < time; i++) {
    vTaskDelay(1000);  
  }
  lockHVAC = false;
  Serial.println("Test complete");

  snprintf_P(data, sizeof(data), PSTR("{\"testRH\":%f}"), humidity);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
}
/////

/////// define function key (2/4)

const std::string fanPowerTopicName = (MQTTCONTROLTOPIC + std::string{"/exhaust"}).data();
const std::string transpirationMeasurement = (MQTTCONTROLTOPIC + std::string{"/transTest"}).data();
//////////



/// @brief  (3/4)
void mqttSubscribeTopics(){
  mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/dehumidifier/auto"}).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/dehumidifier/autoVpd"}).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/dehumidifier/lower"}).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/dehumidifier/upper"}).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/dehumidifier/press"}).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/exhaust"}).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/transTest"}).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/targetVpd"}).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/P"}).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/I"}).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/D"}).data());
}


void mqttHandle(char* topic, String message) {
  
    ////Add to map here (4/4)
    functionDict[fanPowerTopicName] = handleFan;
    functionDict[transpirationMeasurement] = handleTranspirationMeasurement;
    
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
