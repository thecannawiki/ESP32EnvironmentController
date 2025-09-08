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

const void handlelowerBound(const String &message)
{
  float num = message.toFloat();
  lowerHumidityBound = num;
  preferences.putFloat("lowerBound", lowerHumidityBound);
  saveToNVM();
  char data[50];
  snprintf_P(data, sizeof(data), PSTR("{\"lowerBound\":%f}"), lowerHumidityBound);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
  // mqttPublishSensorData();
}

const void handleupperbound(const String &message)
{
  float num = message.toFloat();
  upperHumidityBound = num;
  preferences.putFloat("upperBound", upperHumidityBound);
  saveToNVM();
  char data[50];
  snprintf_P(data, sizeof(data), PSTR("{\"upperBound\":%f}"), upperHumidityBound);
  mqttclient.publish(MQTTPUBLISHTOPIC, data);
  // mqttPublishSensorData();
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

/////

/////// define function key (2/4)
const std::string humidifierStateTopicName = (MQTTCONTROLTOPIC + std::string{"/humidifier"}).data();
const std::string fanPowerTopicName = (MQTTCONTROLTOPIC + std::string{"/exhaust"}).data();
const std::string setHeaterTopicName = (MQTTCONTROLTOPIC + std::string{"/heater"}).data();
const std::string tempTargetTopicName = (MQTTCONTROLTOPIC + std::string{"/heater/target"}).data();
const std::string heaterForTempTopicName = (MQTTCONTROLTOPIC + std::string{"/heater/tempMode"}).data();
const std::string heaterPowerTopicName = (MQTTCONTROLTOPIC + std::string{"/heater/power"}).data();
const std::string fanSoftMax = (MQTTCONTROLTOPIC + std::string{"/exhaust/softMax"}).data();
const std::string fanSoftMin = (MQTTCONTROLTOPIC + std::string{"/exhaust/softMin"}).data();
const std::string transpirationMeasurement = (MQTTCONTROLTOPIC + std::string{"/transTest"}).data();
const std::string dehumidiferToggle = (MQTTCONTROLTOPIC + std::string{"/dehumidifier/toggle"}).data();
const std::string dehumidiferAuto = (MQTTCONTROLTOPIC + std::string{"/dehumidifier/auto"}).data();
const std::string dehumidiferPrimary = (MQTTCONTROLTOPIC + std::string{"/dehumidifier/primary"}).data();
const std::string dehumidiferForTempEndpoint = (MQTTCONTROLTOPIC + std::string{"/dehumidifier/autoTemp"}).data();
const std::string fanAutoVpd = (MQTTCONTROLTOPIC + std::string{"/exhaust/autoVpd"}).data();
const std::string dehumidiferLower = (MQTTCONTROLTOPIC + std::string{"/dehumidifier/lower"}).data();
const std::string dehumidiferUpper = (MQTTCONTROLTOPIC + std::string{"/dehumidifier/upper"}).data();
const std::string setTargetVpd = (MQTTCONTROLTOPIC + std::string{"/targetVpd"}).data();
const std::string setP = (MQTTCONTROLTOPIC + std::string{"/P"}).data();
const std::string setI = (MQTTCONTROLTOPIC + std::string{"/I"}).data();
const std::string setD = (MQTTCONTROLTOPIC + std::string{"/D"}).data();
const std::string pumpTimerTopicName = (MQTTCONTROLTOPIC + std::string{"/pump"}).data();
//////////

/// @brief  (3/4)
void mqttSubscribeTopics()
{
  mqttclient.subscribe(humidifierStateTopicName.data());
  mqttclient.subscribe(dehumidiferAuto.data());
  mqttclient.subscribe(fanAutoVpd.data());
  mqttclient.subscribe(fanSoftMax.data());
  mqttclient.subscribe(fanSoftMin.data());
  mqttclient.subscribe(dehumidiferPrimary.data());
  mqttclient.subscribe(dehumidiferForTempEndpoint.data());
  mqttclient.subscribe(dehumidiferLower.data());
  mqttclient.subscribe(dehumidiferUpper.data());
  mqttclient.subscribe(dehumidiferToggle.data());
  mqttclient.subscribe(fanPowerTopicName.data());
  mqttclient.subscribe(transpirationMeasurement.data());
  mqttclient.subscribe(setTargetVpd.data());
  mqttclient.subscribe(setP.data());
  mqttclient.subscribe(setI.data());
  mqttclient.subscribe(setD.data());
  mqttclient.subscribe(setHeaterTopicName.data());
  mqttclient.subscribe(tempTargetTopicName.data());
  mqttclient.subscribe(heaterForTempTopicName.data());
  mqttclient.subscribe(pumpTimerTopicName.data());
  mqttclient.subscribe(heaterPowerTopicName.data());
}

void mqttHandle(char *topic, String message)
{
  ////Add to map here (4/4) #TODO maybe this can be defined at compile time?
  functionDict[humidifierStateTopicName] = handleHumidifier;
  functionDict[fanPowerTopicName] = handleFan;
  functionDict[transpirationMeasurement] = handleTranspirationMeasurement;
  functionDict[dehumidiferAuto] = handleSetDehumidifierAuto;
  functionDict[dehumidiferPrimary] = handleSetDehumidPrimaryMode;
  functionDict[fanAutoVpd] = handleSetFanAutoVpd;
  functionDict[fanSoftMax] = handleSoftMaxFan;
  functionDict[fanSoftMin] = handleSoftMinFan;
  functionDict[dehumidiferLower] = handlelowerBound;
  functionDict[dehumidiferUpper] = handleupperbound;
  functionDict[dehumidiferToggle] = toggleDehumidifier;
  functionDict[setTargetVpd] = handleTargetVpd;
  functionDict[setP] = handleSetP;
  functionDict[setI] = handleSetI;
  functionDict[setD] = handleSetD;
  functionDict[dehumidiferForTempEndpoint] = handleSetDehumidifierForTemp;
  functionDict[setHeaterTopicName] = handlesetHeater;
  functionDict[pumpTimerTopicName] = handlePumpTimer;
  functionDict[heaterPowerTopicName] = handleHeaterPower;
  functionDict[tempTargetTopicName] = handleTargetTemp;
  functionDict[heaterForTempTopicName] = handleHeaterForTemp;

  // Call the corresponding function based on the input
  std::string topicName = topic;
  auto it = functionDict.find(topicName);
  if (it != functionDict.end())
  {
    // Function found in the dictionary, call it
    std::function<void(const String &)> selectedFunction = it->second;
    selectedFunction(message);
  }
  else
  {
    // Function not found
    Serial.println("Invalid function name!\n");
  }
}
