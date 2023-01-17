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


bool refreshNetworkTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return false;
  }
  
  return true;
}


void initSensors(){
  scd4x.begin(Wire);
  uint16_t error = scd4x.stopPeriodicMeasurement();
  if(error){
    Serial.println("failed to stop SCD40");
  }
  error = scd4x.startPeriodicMeasurement();
  if (error) {
      Serial.print("Error trying to execute startPeriodicMeasurement(): ");
      Serial.println(error);
      SCD40Mounted = false;
  } else {
    Serial.println("SCD-40 connected");
    SCD40Mounted = true;
  }

}


void getSensorReadings(){
  if(SCD40Mounted){
    uint16_t error = scd4x.readMeasurement(co2, temp, humidity);
    Serial.println(co2);
    Serial.println(temp);
    Serial.println(humidity);
    if(error){
      Serial.println("Error reading sensor values from SC40");
      char errorMessage[256];
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
    }
  }
}


float calcVpd(){    //Calc vpd from air temp humidity

  float svp = 610.7 * (pow(10, (7.5*temp/(237.3+temp))));  
  float vpd = (((100 - humidity)/100)*svp)/1000; 

  return vpd;
}

void UpdateSensorJson(){
  StaticJsonDocument<320> doc;  
  char trimmedfloat[10];
  if(temp != -1){
    dtostrf(temp, 5, 4, trimmedfloat);  
    doc["T"] = trimmedfloat;
  }

  if(humidity != -1){
    dtostrf(humidity, 5, 4, trimmedfloat); 
    doc["RH"] = trimmedfloat;
  }
  if(co2 != 65535){doc["co2"] = co2;}   // 65535 is the co2 value from the SCD-40 when its still warming up

  doc["tvoc"] = tvoc;
  dtostrf(calcVpd(), 5, 4, trimmedfloat); 
  doc["vpd"] = trimmedfloat;
  dtostrf(targetVpd, 5, 4, trimmedfloat); 
  doc["targetVpd"] = trimmedfloat;
  doc["autoVpd"] = automaticVpd;
  dtostrf(lowerHumidityBound, 5, 4, trimmedfloat);
  doc["lowerBound"] = trimmedfloat;
  dtostrf(upperHumidityBound, 5, 4, trimmedfloat);
  doc["upperBound"] = trimmedfloat;
  doc["dehumidState"] = dehumidiferState;
  doc["autoDehumid"] = automaticDehumidifier;
  doc["fan"] = fanPower;
  doc["heaterState"] = heaterState;
  doc["ram"] = ESP.getFreeHeap();
  serializeJson(doc, sensorjson);
}

void initNonVolitileMem(){
  //Load params from EEPROM
  preferences.begin("controller", false); 
  fanPower = preferences.getInt("fanPower", 30);
  dehumidiferState = preferences.getBool("dehumidifier", false);
  lowerHumidityBound = preferences.getFloat("lowerBound", 40.0f);
  upperHumidityBound = preferences.getFloat("upperBound", 60.0f);
  automaticDehumidifier = preferences.getBool("autoDehumid", true);
  automaticVpd = preferences.getBool("autoVpd", true);
  targetVpd = preferences.getFloat("tvpd", 1.0f);
  heaterState = preferences.getBool("headerState", false);
  
  UpdateSensorJson();
  Serial.println("retrieved from NVM: ");
  Serial.println(sensorjson);
}

void saveToNVM(){
  preferences.end();
  Serial.print("§ ");
  preferences.begin("controller", false);
}  

void pressDehumidifierButton(){
  pinMode(dehumidifierControlPin, OUTPUT);
  Serial.println("Switching dehumdifier");
  digitalWrite(dehumidifierControlPin, HIGH);  
  delay(500); 
  digitalWrite(dehumidifierControlPin, LOW); 
  dehumidiferState = !dehumidiferState;
  preferences.putBool("dehumidifier", dehumidiferState);
}


//Handlers for web requests
void getSensorReadingsWeb(AsyncWebServerRequest *request){
  UpdateSensorJson();
  request->send_P(200, "application/json", sensorjson);
}

void pressDehumidifierButtonWeb(AsyncWebServerRequest *request){
  pressDehumidifierButton();
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

void setPixelColor(uint32_t color){
  if(LED){
    pixels.setPixelColor(0, color);
    pixels.show();
  }
}

void longPWMloop(void * parameter){
  int totalwidth = 50;
  int highcount = totalwidth * fanPower / 100; 
  int halfPower = (maxPWMval - 1)/ 2 + 1; //sweet trick to divide and get an int
  int count = 0;
  while(true){ 
    if(count < highcount * 2){ //for halfpower change
      ledcWrite(fanPWMchannel, halfPower);
    } else {
      ledcWrite(fanPWMchannel, 0);
    }
    count+=1;
    if(count > totalwidth){
      count = 0;
    }
    vTaskDelay(2000);
  }
}

void setFanPower(int power){ /// 0-100
  if(longPWMTaskHandle != NULL){
    vTaskDelete(longPWMTaskHandle);
  }
  if(power < minpercentvalue){
      ///let the pwm loop task handle the fan
    xTaskCreate(
    longPWMloop,      // Function that should be called
    "longPWM",        // Name of the task (for debugging)
    1000,             // Stack size (bytes)
    NULL,             // Parameter to pass
    3,                // Task priority
    &longPWMTaskHandle);       // Task handle 
    Serial.println("made new fan task");
  } else {
    int powerVal = map(power, 0, 100, 0, maxPWMval);
    ledcWrite(fanPWMchannel, powerVal);
  }

  esp_task_wdt_reset();
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


//neopixel animations
void rbgloop(void * parameter){
  int i = 0;
  int totalLoopCounter = 0;
  for(;;){ // infinite loop
    
    switch (i){
      case 0:
        setPixelColor(pixels.Color(255,0,0));
        break;
      case 1:
        setPixelColor(pixels.Color(0,255,0));
        break;
      case 2:
        setPixelColor(pixels.Color(0,0,255));
        break;
    }

    i++;
    totalLoopCounter++;
    if (i>2){
      i=0;
    }
    vTaskDelay(800);
    if (totalLoopCounter > 7){
      setPixelColor(pixels.Color(255, 153, 51)); //orange
      vTaskDelay(1000);
      ESP.restart();
    }
  }
}

void mqttconnectloop(void * parameter){
  bool col = true;
  for(;;){ // infinite loop
    
    if (col){
        setPixelColor(pixels.Color(14, 218, 240));  //blue
      }else{
        setPixelColor(pixels.Color(7, 50, 224));    //differnet blue
    }

    col = !col;
    vTaskDelay(1000);
  }
}

void flash3green(){
  int delayTime = 90;
  int onTime = 120;
  setPixelColor(pixels.Color(0, 0, 0));
  delay(300);
  setPixelColor(pixels.Color(0,255,0));
  delay(onTime);
  setPixelColor(pixels.Color(0, 0, 0));
  delay(delayTime);
  setPixelColor(pixels.Color(0,255,0));
  delay(onTime);
  setPixelColor(pixels.Color(0, 0, 0));
  delay(delayTime);
  setPixelColor(pixels.Color(0,255,0));
  delay(onTime);
  setPixelColor(pixels.Color(0, 0, 0));
}

void startLedanimation(void (*func)(void *)){
  if(LedAnimationTaskHandle != NULL){
    vTaskDelete(LedAnimationTaskHandle);
  }
  xTaskCreate(func, "loop", 1000, 0, 0, &LedAnimationTaskHandle);  
}


void mqttreconnect() {    //TODO check mqtt stuff is defined
  
  int errcount = 0;
  // Loop until we're reconnected
  while (!mqttclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    String clientId = "esp32";

    if (mqttclient.connect(clientId.c_str(), MQTTUSER, MQTTPASS)) {
      Serial.println("connected");
      
      // Subscribe
      mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/dehumidifier/auto"}).data());
      mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/dehumidifier/autoVpd"}).data());
      mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/dehumidifier/lower"}).data());
      mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/dehumidifier/upper"}).data());
      mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/dehumidifier/target"}).data());
      mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/dehumidifier/press"}).data());
      mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/exhaust"}).data());
      mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/heater"}).data());
      mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/vibe"}).data());
      mqttclient.subscribe((MQTTCONTROLTOPIC + std::string{"/targetVpd"}).data());
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttclient.state());
      Serial.println(" try again in 5 seconds");
      errcount++;
      if(errcount>3){
        ESP.restart();
      }
      // Wait 5 seconds before retrying
      delay(5000);
      
    }
  }
}

void stirPattern(int length){
    digitalWrite(stirrerControlPin, HIGH); 
    delay(length);
    digitalWrite(stirrerControlPin, LOW); 
}

void mqttPublishSensorData(){ //TODO check if mqtt in creds
  // if(!mqttclient.connected()){
  //   mqttreconnect();
  // }
  UpdateSensorJson();
  bool success = mqttclient.publish(MQTTPUBLISHTOPIC, sensorjson, false);
  if(success){
    Serial.print("-> ");
  } else {
    Serial.print("MQTT send failed with"); Serial.println(sensorjson);
  }
  
}

void mqttMessageReceived(char* topic, byte* message, unsigned int length) {
  esp_task_wdt_reset();
  String messageTemp;
  Serial.println("");
  
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }

  Serial.print("~#");
  Serial.print(topic);
  Serial.print(" | ");
  Serial.println(messageTemp);

  if (String(topic) == "box/control/exhaust") {
    int num = messageTemp.toInt();
    if(num>100){ num = 100;}
    if(num<0){num = 0;}
    if(num != fanPower){
      fanChanged = true;
    }
    fanPower = num;
    preferences.putInt("fanPower", fanPower);
    saveToNVM();

    char data[20];
    snprintf_P(data, sizeof(data), PSTR("{\"fan\":%i}"), fanPower);
    mqttclient.publish(MQTTPUBLISHTOPIC, data);

    return;
  }


  if (String(topic) == "box/control/dehumidifier/press") {
    pressDehumidifierButton();
    return;
  }

  if (String(topic) == "box/control/vibe") {
    int num = messageTemp.toInt();    
    if(num > 10000){num = 10000;}
    stirPattern(num);
    return;
  }

  if(String(topic) == "box/control/targetVpd"){
    float num = messageTemp.toFloat();
    if(num <= 2.0f && num >=0.4f){
      targetVpd = num;
      preferences.putFloat("tvpd", targetVpd);
      saveToNVM();
    }
    return;
  }

  if (String(topic) == "box/control/dehumidifier/auto") {
    
    if(messageTemp == "on"){
      automaticDehumidifier = true;
      Serial.println("automatic dehumidifer control enabled");
    }
    else if(messageTemp == "off"){
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
    return;
  }

  if (String(topic) == "box/control/dehumidifier/autoVpd") {
    
    if(messageTemp == "on"){
      automaticVpd = true;
    }
    else if(messageTemp == "off"){
      automaticVpd = false;
    }
    preferences.putBool("autoVpd", automaticVpd);
    saveToNVM();

    char data[50];
    snprintf_P(data, sizeof(data), PSTR("{\"autoVpd\":%d}"), automaticVpd);
    mqttclient.publish(MQTTPUBLISHTOPIC, data);
    return;
  }

  if (String(topic) == "box/control/dehumidifier/lower") {
    float num = messageTemp.toFloat();
    lowerHumidityBound = num;
    preferences.putFloat("lowerBound", lowerHumidityBound);
    saveToNVM();
    char data[50];
    snprintf_P(data, sizeof(data), PSTR("{\"lowerBound\":%f}"), lowerHumidityBound);
    mqttclient.publish(MQTTPUBLISHTOPIC, data);
    //mqttPublishSensorData();
    return;
  }

  if (String(topic) == "box/control/dehumidifier/upper") {
    float num = messageTemp.toFloat();
    upperHumidityBound = num;
    preferences.putFloat("upperBound", upperHumidityBound);
    saveToNVM();
    char data[50];
    snprintf_P(data, sizeof(data), PSTR("{\"upperBound\":%f}"), upperHumidityBound);
    mqttclient.publish(MQTTPUBLISHTOPIC, data);
    //mqttPublishSensorData();
    return;
  }

  if (String(topic) == "box/control/heater") {
    if(messageTemp == "on"){
      heaterState = true;
      digitalWrite(heaterControlPin, HIGH); 
    }
    else if(messageTemp == "off"){
      heaterState = false;
      digitalWrite(heaterControlPin, LOW); 
    }
    preferences.putBool("headerState", heaterState);
    saveToNVM();
    char data[50];
    snprintf_P(data, sizeof(data), PSTR("{\"heaterState\":%d}"), heaterState);
    mqttclient.publish(MQTTPUBLISHTOPIC, data);
    //mqttPublishSensorData();
    return;
  }
}

void operateDehumidifier(){   //if humidity too high turn off the dehumifier and vice-versa
  if(SCD40Mounted && humidity>=(float)0){
    Serial.print("O ");
    if(dehumidiferState){ //dehumidifier on
        if(humidity <= lowerHumidityBound){
          pressDehumidifierButton();
        } else{
          //Serial.println("dehumdifier in the right state");
        }
    } else{ //dehumidifer off
      if(humidity >= upperHumidityBound){
        pressDehumidifierButton();
      } else{
        //Serial.println("dehumdifier in the right state");
      }
    }
  }
}


void mainloop(void * parameter){
  vTaskDelay(1000);
  while(true){
    if(fanChanged){
      setFanPower(fanPower);
      fanChanged=false;
    }
    
    if(loopCounter%2 == 0){
    getSensorReadings();
    }

    if(loopCounter%5 == 0){    //every sensorInterval * num seconds i%num 
      if(automaticDehumidifier){
        if(automaticVpd){
          float hu  = calcTargetHumidityForVpd(targetVpd, temp, humidity);
          if(hu != -1){
            Serial.print("±RH ");
            lowerHumidityBound = hu - 1.0f;
            upperHumidityBound = hu + 1.0f;
            preferences.putInt("lowerBound", lowerHumidityBound);
            preferences.putInt("higherBound", upperHumidityBound);
            saveToNVM();
          } 
        }
        operateDehumidifier();
      }
      if(mqttclient.connected()){
        mqttPublishSensorData();
      }
    }

    vTaskDelay(sensorInterval);
    loopCounter++;    //changing this value resets the freezewatchdog
    if(loopCounter > 1000){
      loopCounter = 0;
    }
  }
}

void mqttLoop(void * parameter){
  Serial.println("");
  Serial.println("mqtt client listening");
  while(true){
    if(mqttclient.connected()){
      Serial.print("¤ ");
     mqttclient.loop();
    } else {
      mqttreconnect();
      mqttclient.loop();
    }
    vTaskDelay(1000);
  }
}


void setup() {
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector 
  Serial.begin(115200);
  while (!Serial) { delay(100); } // Wait for serial console to open!
  delay(1000);
  Wire.begin();
  Wire.setClock(10000);
  xTaskCreate(freezeWatchdog, "watchdog", 1000, 0, 0, &freezewatchdogTaskHandle);

  if(LED){
    pixels.begin();
    pixels.setBrightness(30);
    startLedanimation(rbgloop);
  }

  ledcSetup(fanPWMchannel, freq, resolution);
  ledcAttachPin(fanControlPin, fanPWMchannel);

  pinMode(heaterControlPin, OUTPUT);
  digitalWrite(heaterControlPin, LOW); 
  pinMode(stirrerControlPin, OUTPUT);
  digitalWrite(stirrerControlPin, LOW); 
  
  //sensors
  initSensors();

  //get NVM
  initNonVolitileMem();
  Serial.println("");
  delay(200);

  //Connect wifi
  Serial.println("starting WIFI..");
  wifiMulti.addAP(SSIDNAME, WIFIPASS);    //Add additional networks here
  wifiMulti.run();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(2000);
  }
  Serial.print("Connected to: ");
  Serial.println(SSIDNAME);
  Serial.println(WiFi.RSSI());
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

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
  
  xTaskCreate(mqttLoop, "mqttHandler", 4000, 0, 1, &mqttTaskHandle);
  xTaskCreate(mainloop, "main", 90000, 0, 0, &mainLoopTaskHandle);
  flash3green();
  setFanPower(fanPower);
}


//not used
void loop(){}