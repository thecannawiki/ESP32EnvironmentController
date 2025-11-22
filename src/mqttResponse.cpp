#include <string>
#include <unordered_map>
#include <functional>
#include <Credentials.h>
#include <globals.h>
#include <mathsfunctions.h>
#include <NVM.h>
#include <peripherals.h>
std::unordered_map<std::string, std::function<void(const String &)>> functionDict;

const bool onOffToBool(const String &message)
{
  if (message == "on")
  {
    return true;
  }
  else if (message == "off")
  {
    return false;
  }
  // oop idk what to do here ??
  return false;
}

// Function definitions (1/4)
const void handleFan(const String &message)
{
  if (!lockHVAC)
  {
    float num = message.toFloat();
    if (num > softMaxFan){ num = softMaxFan;}
    if (num < softMinFan){ num = softMinFan;}
    if (num > 100)
    {
      num = 100;
    }
    if (num < 0)
    {
      num = 0;
    }

    if (num != fanPower)
    {
      fanPower = num;
      fanChanged = true;
      Serial.println("Fan power changed!");
    }
    preferences.putFloat("fanPower", fanPower);
    saveToNVM();

    char data[20];
    snprintf_P(data, sizeof(data), PSTR("{\"fan\":%f}"), fanPower);
    mqttclient.publish(MQTTPUBLISHTOPIC, data);
  }
  else
  {
    Serial.println("oop HVAC controls locked");
  }
}

const void handleWaterThresh(const String &message){
  int num = message.toInt();
  if (num < 0 || num > 4045)
  {
    return;
  }

  w1maxWaterSensorVal = num;
  preferences.putInt("w1Max", w1maxWaterSensorVal);
  saveToNVM();

  Serial.println(w1maxWaterSensorVal);

  char data[60];
  snprintf_P(data, sizeof(data), PSTR("{\"w1T\":%d}"), w1maxWaterSensorVal);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
}

const void handlePumpTimer(const String &message)
{
  int num = message.toInt();
  if (num > 35 * 60)
  { // 35 mins
    Serial.println("pump time too long");
    return;
  }
  if (num < 0)
  {
    return;
  }

  if (pumpEnd != 0)
  {
    Serial.println("pump timer already set");
    return;
  }

  if (waterSensor1State > w1maxWaterSensorVal)
  {
    Serial.println("Water sensor 1 detects water! pump disabled");
    char data[60];
    snprintf_P(data, sizeof(data), PSTR("{\"pump_time_active\":%f,\"early_stop\":%d}"), 0.0f, bool(true));
    mqttclient.publish(MQTTPUBLISHTOPIC, data);
    return;
  }

  time(&timeNow);
  Serial.print("epoch now: ");
  Serial.println(timeNow);

  pumpState = true;
  digitalWrite(pumpControlPin, HIGH);
  pumpEnd = timeNow + num;
  pumpStart = timeNow;
  Serial.println("Pump timer started");
  Serial.print("end time: ");
  Serial.println(pumpEnd);
}

const void handleSoftMaxFan(const String &message)
{
  float num = message.toFloat();
  if (num > 100)
  {
    num = 100;
  }
  if (num < 0)
  {
    num = 0;
  }

  softMaxFan = num;
  if (fanPower > num)
  {
    fanPower = num;
    fanChanged = true;
  }

  preferences.putFloat("softMaxFan", num);
  saveToNVM();

  char data[30];
  snprintf_P(data, sizeof(data), PSTR("{\"softMaxFan\":%f}"), num);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
}

const void handleSoftMinFan(const String &message)
{
  float num = message.toFloat();
  if (num > 100)
  {
    num = 100;
  }
  if (num < 0)
  {
    num = 0;
  }

  softMinFan = num;
  if (fanPower > num)
  {
    fanPower = num;
    fanChanged = true;
  }

  preferences.putFloat("softMinFan", num);
  saveToNVM();

  char data[30];
  snprintf_P(data, sizeof(data), PSTR("{\"softMinFan\":%f}"), num);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
}

void transpirationTestTask(void *message)
{
  int time = transTestTime;
  Serial.print("trans test ");
  Serial.print(time);

  char data[60];
  float oldFanPower = fanPower;

  Serial.println("Starting transpiration test");
  lockHVAC = true;
  fanPower = 0;
  fanChanged = true;

  float startWater = calcGramsOfWaterInAir(temp, humidity);
  Serial.print("sending trans test mqtt");
  snprintf_P(data, sizeof(data), PSTR("{\"testRH1\":%f,\"water1\":%f}"), humidity, startWater);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);

  Serial.print("waiting");
  for (int i = 0; i < time; i++)
  {
    vTaskDelay(1000);
    Serial.println("");
    Serial.print("transpiration test ongoing:");
    Serial.print(i);
    Serial.print("/");
    Serial.println(time);
  }
  lockHVAC = false;
  Serial.println("Test complete");
  fanPower = oldFanPower;
  fanChanged = true;

  float endWater = calcGramsOfWaterInAir(temp, humidity);
  snprintf_P(data, sizeof(data), PSTR("{\"waterDiff\":%f,\"water2\":%f}"), endWater - startWater, endWater);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
  vTaskDelete(NULL);
}

const void handleTranspirationMeasurement(const String &message)
{
  int time = message.toInt();

  if (time < 1)
  {
    mqttclient.publish(MQTTPUBLISHTOPIC, "time too short");
    return;
  }
  if (time > 300)
  {
    mqttclient.publish(MQTTPUBLISHTOPIC, "time too long");
    Serial.print("trans test time to long");
    return;
  }
  if(!(SCD40Mounted || bmeMounted)){
    Serial.println("No sensors mounted for trans test");
    return;
  }

  if (!lockHVAC)
  { // Test could already be running
    transTestTime = time;
    xTaskCreate(
        transpirationTestTask, // Function that should be called
        "transTest",           // Name of the task (for debugging)
        1886,                  // Stack size (bytes)
        0,                     // Parameter to pass
        3,                     // Task priority
        &TransTestTaskHandle); // Task handle
  }
  else
  {
    Serial.println("Can't start trans test with lockHVAC");
  }
}

const void toggleDehumidifier(const String &message)
{
  if (!lockHVAC)
  {
    setDehumidifierState(!dehumidiferState);
  }
}

const void handleTargetVpd(const String &message)
{
  float num = message.toFloat();
  if (num <= 3.0f && num >= 0.5f)
  {
    targetVpd = num;
    preferences.putFloat("tvpd", targetVpd);
    saveToNVM();
  }
}

const void handleTargetHumidity(const String &message)
{
  float num = message.toFloat();
  targetHumidity = num;
  preferences.putFloat("targetHumidity", targetHumidity);
  saveToNVM();
  char data[50];
  snprintf_P(data, sizeof(data), PSTR("{\"targetRH\":%f}"), targetHumidity);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
}

const void handleHeaterPower(const String &message)
{
  int num = message.toInt();
  if (num > 0 && num <= 100)
  {
    heaterPower = num;
    // preferences.putFloat("=", targetTemperature);
    // saveToNVM();
    char data[60];
    snprintf_P(data, sizeof(data), PSTR("{\"heaterPower\":%f}"), heaterPower);
    mqttclient.publish(MQTTPUBLISHTOPIC, data);
    setHeaterState(heaterState); // update heater hardware
  }
}

const void handleTargetTemp(const String &message)
{
  float num = message.toFloat();
  if (num <= 30.0f && num >= 15.0f)
  {
    targetTemperature = num;
    preferences.putFloat("ttemp", targetTemperature);
    saveToNVM();
    char data[60];
    snprintf_P(data, sizeof(data), PSTR("{\"targetTemp\":%f}"), targetTemperature);
    mqttclient.publish(MQTTPUBLISHTOPIC, data);
  }
}

const void handleHeaterForTemp(const String &message)
{
  bool val = onOffToBool(message);
  heaterTempMode = val;
  preferences.putFloat("HFT", heaterTempMode);
  saveToNVM();
  char data[60];
  snprintf_P(data, sizeof(data), PSTR("{\"HeaterForTemp\":%d}"), heaterTempMode);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
}

const void handleSetP(const String &message)
{
  float num = message.toFloat();
  P = num;
  preferences.putFloat("P", P);
  saveToNVM();
}

const void handleSetI(const String &message)
{
  float num = message.toFloat();
  I = num;
  preferences.putFloat("I", I);
  saveToNVM();
}

const void handleSetD(const String &message)
{
  float num = message.toFloat();
  D = num;
  preferences.putFloat("D", D);
  saveToNVM();
}

const void handleSetDehumidifierAuto(const String &message)
{
  if (message == "on")
  {
    automaticDehumidifier = true;
    Serial.println("automatic dehumidifer control enabled");
  }
  else if (message == "off")
  {
    automaticDehumidifier = false;
    Serial.println("automatic dehumidifer control disabled");
  }
  preferences.putBool("autoDehumid", automaticDehumidifier);
  saveToNVM();

  char data[50];
  snprintf_P(data, sizeof(data), PSTR("{\"autoDehumid\":%d}"), automaticDehumidifier);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
  // mqttPublishSensorData();
}

const void handleSetFanAutoVpd(const String &message)
{
  if (message == "on")
  {
    automaticFanVpd = true;
  }
  else if (message == "off")
  {
    automaticFanVpd = false;
  }
  preferences.putBool("autoVpd", automaticFanVpd);
  saveToNVM();

  char data[50];
  snprintf_P(data, sizeof(data), PSTR("{\"autoVpd\":%d}"), automaticFanVpd);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
}

const void handleSetDehumidPrimaryMode(const String &message)
{
  if (message == "on")
  {
    dehumidifierPrimaryMode = true;
  }
  else if (message == "off")
  {
    dehumidifierPrimaryMode = false;
  }
  preferences.putBool("primaryHumid", dehumidifierPrimaryMode);
  saveToNVM();

  char data[50];
  snprintf_P(data, sizeof(data), PSTR("{\"primaryHumid\":%d}"), dehumidifierPrimaryMode);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
}




const void handlesetHeater(const String &message)
{
  if (message == "on")
  {
    setHeaterState(true);
  }
  else if (message == "off")
  {
    setHeaterState(false);
  }

  if (message == "auto")
  {
    autoHeater = true;
  }
  else if (message == "man")
  {
    autoHeater = false;
  }

  preferences.putBool("headerState", heaterState);
  preferences.putBool("autoHeater", autoHeater);

  saveToNVM();
  char data[50];
  snprintf_P(data, sizeof(data), PSTR("{\"heater\":%d, \"autoHeater\":%d}"), heaterState, autoHeater);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
}

const void handleSetDehumidifierForTemp(const String &message)
{
  if (onOffToBool(message))
  {
    dehumidifierForTemp = true;
    dehumidifierPrimaryMode = false;
  }
  else
  {
    dehumidifierForTemp = false;
  }

  preferences.putBool("primaryHumid", dehumidifierPrimaryMode);
  preferences.putBool("dehumidAutoTemp", dehumidifierForTemp);
  saveToNVM();
  char data[50];
  snprintf_P(data, sizeof(data), PSTR("{\"dehumidAutoTemp\":%d, \"primaryHumid\":%d}"), dehumidifierForTemp, dehumidifierPrimaryMode);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
}

const void handleHumidifier(const String &message){
  if(message == "auto"){
    automaticHumidifier = true;
    preferences.putBool("AHU", automaticHumidifier);
    saveToNVM();
    char data[20];
    snprintf_P(data, sizeof(data), PSTR("{\"AHU\":%d}"), automaticHumidifier);
    mqttclient.publish(MQTTPUBLISHTOPIC, data);
    return;
  } 
  if(message == "man"){
    automaticHumidifier = false;
    preferences.putBool("AHU", automaticHumidifier);
    saveToNVM();
    char data[20];
    snprintf_P(data, sizeof(data), PSTR("{\"AHU\":%d}"), automaticHumidifier);
    mqttclient.publish(MQTTPUBLISHTOPIC, data);
    return;
  }
  
  bool val = onOffToBool(message);
  setHumidifierState(val);
  saveToNVM();
  char data[20];
  snprintf_P(data, sizeof(data), PSTR("{\"HU\":%d,\"AHU\":%d}"), humidifierState, automaticHumidifier);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
}

const void handleAutoControlMode(const String &message){
  if(message == "VPD"){
    vpdMode=true;
  }

  else if(message == "RH"){
    vpdMode=false;
  }

  preferences.putBool("VpdMode", vpdMode);
  saveToNVM();
  char data[20];
  snprintf_P(data, sizeof(data), PSTR("{\"VPDMode\":%d}"), vpdMode);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);

}

/////

/////// define function key (2/4)
const std::string humidifierStateTopicName = "/humidifier";
const std::string fanPowerTopicName = "/exhaust";
const std::string setHeaterTopicName = "/heater";
const std::string tempTargetTopicName = "/heater/target";
const std::string heaterForTempTopicName = "/heater/tempMode";
const std::string heaterPowerTopicName = "/heater/power";
const std::string fanSoftMax = "/exhaust/softMax";
const std::string fanSoftMin ="/exhaust/softMin";
const std::string transpirationMeasurement = "/transTest";
const std::string dehumidiferToggle = "/dehumidifier/toggle";
const std::string dehumidiferAuto = "/dehumidifier/auto";
const std::string dehumidiferPrimary = "/dehumidifier/primary";
const std::string dehumidiferForTempEndpoint = "/dehumidifier/autoTemp";
const std::string fanAutoVpd = "/exhaust/autoVpd";
const std::string setTargetVpd = "/targetVpd";
const std::string setP = "/P";
const std::string setI = "/I";
const std::string setD ="/D";
const std::string pumpTimerTopicName = "/pump";
const std::string w1ThreshTopicName ="/w1T";
const std::string controlModeTopicName = "/auto/mode";
const std::string handleTargetHumidityTopicName = "/targetRH";

//////////

/// @brief  (3/4)
void mqttSubscribeTopics(std::string MQTTCONTROLTOPIC)
{
  mqttclient.subscribe("help");
  mqttclient.subscribe((MQTTCONTROLTOPIC + humidifierStateTopicName).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + dehumidiferAuto).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + fanAutoVpd).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + fanSoftMax).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + fanSoftMin).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + dehumidiferPrimary).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + dehumidiferForTempEndpoint).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + dehumidiferToggle).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + fanPowerTopicName).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + transpirationMeasurement).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + setTargetVpd).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + setP).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + setI).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + setD).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + setHeaterTopicName).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + tempTargetTopicName).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + heaterForTempTopicName).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + pumpTimerTopicName).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + heaterPowerTopicName).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + controlModeTopicName).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + handleTargetHumidityTopicName).data());
  mqttclient.subscribe((MQTTCONTROLTOPIC + w1ThreshTopicName).data());
}

void mqttHandle(char *topic, String message)
{
  String topicStr = String(topic);
  const String prefix = MQTTCONTROLTOPIC;

  // Remove the common prefix once
  if (topicStr.startsWith(prefix)) {
      topicStr.remove(0, prefix.length()); // subtract prefix
  }
  ////Add to map here (4/4) 
  functionDict[humidifierStateTopicName.c_str()] = handleHumidifier;
  functionDict[fanPowerTopicName.c_str()] = handleFan;
  functionDict[transpirationMeasurement.c_str()] = handleTranspirationMeasurement;
  functionDict[dehumidiferAuto.c_str()] = handleSetDehumidifierAuto;
  functionDict[dehumidiferPrimary.c_str()] = handleSetDehumidPrimaryMode;
  functionDict[fanAutoVpd.c_str()] = handleSetFanAutoVpd;
  functionDict[fanSoftMax.c_str()] = handleSoftMaxFan;
  functionDict[fanSoftMin.c_str()] = handleSoftMinFan;
  functionDict[dehumidiferToggle.c_str()] = toggleDehumidifier;
  functionDict[setTargetVpd.c_str()] = handleTargetVpd;
  functionDict[setP.c_str()] = handleSetP;
  functionDict[setI.c_str()] = handleSetI;
  functionDict[setD.c_str()] = handleSetD;
  functionDict[dehumidiferForTempEndpoint.c_str()] = handleSetDehumidifierForTemp;
  functionDict[setHeaterTopicName.c_str()] = handlesetHeater;
  functionDict[pumpTimerTopicName.c_str()] = handlePumpTimer;
  functionDict[heaterPowerTopicName.c_str()] = handleHeaterPower;
  functionDict[tempTargetTopicName.c_str()] = handleTargetTemp;
  functionDict[heaterForTempTopicName.c_str()] = handleHeaterForTemp;
  functionDict[controlModeTopicName.c_str()] = handleAutoControlMode;
  functionDict[handleTargetHumidityTopicName.c_str()] = handleTargetHumidity;
  functionDict[w1ThreshTopicName.c_str()] = handleWaterThresh;

  // Call the corresponding function based on the input
  // std::string topicName = topic;
  auto it = functionDict.find(topicStr.c_str());
  if (it != functionDict.end())
  {
    // Function found in the dictionary, call it
    std::function<void(const String &)> selectedFunction = it->second;
    selectedFunction(message);
  }
  else
  {
    Serial.println("Handler function not found!\n");
  }
}
