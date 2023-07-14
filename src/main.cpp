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
#ifdef BME
  #include <bme.h>
#endif
#ifdef SCD4
  #include <scd4.h>
#endif


int PIDLookback = 12; //number of samples to look back on for PID 



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
  dtostrf(calcVpd(temp, humidity), 5, 4, trimmedfloat); 
  doc["vpd"] = trimmedfloat;
  dtostrf(targetVpd, 5, 4, trimmedfloat); 
  doc["targetVpd"] = trimmedfloat;
  doc["autoVpd"] = automaticVpd;
  dtostrf(targetHumidity, 5, 4, trimmedfloat);
  doc["targetHu"] = trimmedfloat;
  dtostrf(P, 5, 4, trimmedfloat);
  doc["P"] = trimmedfloat;
  dtostrf(I, 5, 4, trimmedfloat);
  doc["I"] = trimmedfloat;
  dtostrf(D, 5, 4, trimmedfloat);
  doc["D"] = trimmedfloat;
  dtostrf(fanPower, 5, 1, trimmedfloat);
  doc["fan"] = trimmedfloat;
  // dtostrf(lowerHumidityBound, 5, 4, trimmedfloat);
  // doc["lowerBound"] = trimmedfloat;
  // dtostrf(upperHumidityBound, 5, 4, trimmedfloat);
  // doc["upperBound"] = trimmedfloat;
  doc["dehumidState"] = dehumidiferState;
  doc["autoDehumid"] = automaticDehumidifier;
  
  //doc["heaterState"] = heaterState;
  //doc["ram"] = ESP.getFreeHeap();
  serializeJson(doc, sensorjson);
}

void initNonVolitileMem(){
  preferences.begin("controller", false); 
  fanPower = preferences.getFloat("fanPower", 0.0f);
  dehumidiferState = preferences.getBool("dehumidifier", false);
  lowerHumidityBound = preferences.getFloat("lowerBound", 40.0f);
  upperHumidityBound = preferences.getFloat("upperBound", 60.0f);
  P = preferences.getFloat("P", 0.2f);
  I = preferences.getFloat("I", 0.2f);
  D = preferences.getFloat("D", 10.0f);
  automaticDehumidifier = preferences.getBool("autoDehumid", true);
  automaticVpd = preferences.getBool("autoVpd", true);
  targetVpd = preferences.getFloat("tvpd", 1.0f);
  heaterState = preferences.getBool("headerState", false);
  
  UpdateSensorJson();
  Serial.println("retrieved from NVM: ");
  Serial.println(sensorjson);
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



void longPWMloop(void *parameter){ // 0-1000
  int *power;
  power = (int *) parameter;
  int totalwidth = 50;
  int highcount = totalwidth * int(power) / 1000; 
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
    if(fanChanged){  vTaskDelete( NULL ); Serial.println("fan task killed");}
    vTaskDelay(2000);
  }
}

void updateFanPower(){ /// 0-100 to 1 dp. Value then gets converted to int 0-1000 by multiplying by 10

  int powerVal  = fanPower * 10;
  float percentVal = powerVal / 1000;

  if(powerVal>1000){powerVal = 1000;}
  if(powerVal<0){powerVal = 0;}

  if(percentVal < minpercentvalue){
      ///let the pwm loop task handle the fan
    xTaskCreate(
    longPWMloop,      // Function that should be called
    "longPWM",        // Name of the task (for debugging)
    1000,             // Stack size (bytes)
    (void *) powerVal,// Parameter to pass
    3,                // Task priority
    &longPWMTaskHandle);       // Task handle 
    //Serial.print("made new fan task. fan value ");
    //Serial.println(powerVal);
  } else {
    powerVal = map(powerVal, 0, 1000, 0, maxPWMval);
    ledcWrite(fanPWMchannel, powerVal);
  }

  preferences.putFloat("fanPower", fanPower);
  saveToNVM();
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
  while (WiFi.status() != WL_CONNECTED) {
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



  if (String(topic) == (MQTTCONTROLTOPIC + std::string{"/dehumidifier/toggle"}).data()) {
    if(!lockHVAC){
      pressDehumidifierButton();
    }
    return;
  }


  if(String(topic) == (MQTTCONTROLTOPIC + std::string{"/targetVpd"}).data()){
    float num = messageTemp.toFloat();
    if(num <= 2.0f && num >=0.4f){
      targetVpd = num;
      preferences.putFloat("tvpd", targetVpd);
      saveToNVM();
    }
    return;
  }

  //PID stuff
  if(String(topic) == (MQTTCONTROLTOPIC + std::string{"/P"}).data()){
    float num = messageTemp.toFloat();
      P = num;
      preferences.putFloat("P", P);
      saveToNVM();
    return;
  }
  if(String(topic) == (MQTTCONTROLTOPIC + std::string{"/I"}).data()){
    float num = messageTemp.toFloat();
      I = num;
      preferences.putFloat("I", I);
      saveToNVM();
    return;
  }
  if(String(topic) == (MQTTCONTROLTOPIC + std::string{"/D"}).data()){
    float num = messageTemp.toFloat();
      D = num;
      preferences.putFloat("D", D);
      saveToNVM();
    return;
  }


  if (String(topic) == (MQTTCONTROLTOPIC + std::string{"/dehumidifier/auto"}).data()) {
    
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

  if (String(topic) == (MQTTCONTROLTOPIC + std::string{"/dehumidifier/autoVpd"}).data()) {
    
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

  if (String(topic) == (MQTTCONTROLTOPIC + std::string{"/dehumidifier/lower"}).data()) {
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

  if (String(topic) == (MQTTCONTROLTOPIC + std::string{"/dehumidifier/upper"}).data()) {
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

  if (String(topic) == (MQTTCONTROLTOPIC + std::string{"/heater"}).data()) {
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

void operateDehumidifier(){   //if humidity too high turn on the dehumidifier and vice-versa
  if(humidity>=(float)-1){
    Serial.print("O ");
    if(dehumidiferState){ //dehumidifier on
        if(humidity <= lowerHumidityBound){
          pressDehumidifierButton();
        }
    } else{ //dehumidifer off
      if(humidity >= upperHumidityBound){
        pressDehumidifierButton();
      }
    }
  }
}

float calcDfan(){
  // Serial.println("buffer");
  // for(int i=0; i<BUFFER_SIZE; i++){
  //   Serial.println(buffer.data[i]);
  // }

  int tenindex = humidityBuffer.newest_index-PIDLookback;
  int cindex = humidityBuffer.newest_index-1;
  if(tenindex < 0){
      tenindex = BUFFER_SIZE + tenindex;
  }
  if(cindex < 0){
      cindex = BUFFER_SIZE + cindex;
  }
  float old = humidityBuffer.data[tenindex];
  float current = humidityBuffer.data[cindex];
   if(old ==-1.0f || old == 0.0f || current ==-1.0f || current== 0.0f){
      return 0;
    }

  float diff = ( current - old) / PIDLookback;
  
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
  for(int i=1; i<PIDLookback; i++){
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
      if(newfan <=0){newfan = 0.1f;}
      if(newfan >=100){newfan = 100;}
      if(abs(newfan - fanPower) >= 0.1f){
       
        Serial.print("new fan power ");
        Serial.println(newfan);
        fanPower=newfan;
        fanChanged=true;
      }
}

void checkSensors(){
  float num = 0;
  int dupecount =0;
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


void mainloop(void * parameter){
  vTaskDelay(1000);
  int sensorTime = 5; //num of loop interations between sensor reading/PID. Every x loops
  #ifdef BME
    sensorTime = 1;
    sensorInterval = 1000;
  #endif

  while(true){
    if(fanChanged){           //cool trick to recreate fan task at a good time (where it wont crash)
      updateFanPower();
      fanChanged=false;
    }

    if(lockHVAC && dehumidiferState){   //turn off dehumidifer when HVAC is locked (not perfect as this is now a sideffect of the var)
      pressDehumidifierButton();
    }
   
    
    if(loopCounter%sensorTime == 0){
      checkSensors();
      getSensorReadings();
      if(!lockHVAC){
        fanPID();
        if(automaticDehumidifier){
          operateDehumidifier();
        }
      }
    }

    if(loopCounter%10 == 0){
      Serial.print("checking wifi ..");
      if(WiFi.status() != WL_CONNECTED){
        bool success = WiFi.reconnect();
        if(success == ESP_OK){
          Serial.println("connected");
        }
        Serial.println("failed");
      }
    }

    if(loopCounter%5 == 0){    //every sensorInterval * num seconds i%num (10 secs)
      if(automaticVpd){
        targetHumidity = calcTargetHumidityForVpd(targetVpd, temp);
        Serial.print("±RH ");
        lowerHumidityBound = targetHumidity - 0.5f;
        upperHumidityBound = targetHumidity + 0.5f;
      }
      

      if(WiFi.status() == WL_CONNECTED && mqttclient.connected()){
        mqttPublishSensorData();
      }

    }
    if(loopCounter %30 == 0){
      tm time = refreshNetworkTime();
      Serial.println("");
      Serial.print(time.tm_hour);
      Serial.print(":");
      Serial.print(time.tm_min);
      Serial.println("");
    }

    
    vTaskDelay(sensorInterval);
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

  ledcSetup(fanPWMchannel, freq, resolution);
  ledcAttachPin(fanControlPin, fanPWMchannel);

  pinMode(heaterControlPin, OUTPUT);
  digitalWrite(heaterControlPin, LOW); 
  pinMode(stirrerControlPin, OUTPUT);
  digitalWrite(stirrerControlPin, LOW); 
  
  //sensors
  initSensors();
  delay(1000);

  //get NVM
  initNonVolitileMem();
  Serial.println("");
  delay(200);

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