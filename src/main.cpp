/**********

Hardware support and default pins
  * 1 PWM fan   12
  * 1 dehumidifier 13
  * 1 heater 33
  * vibrator 26
  * neopixel 19

*********/
//lots of libraries
#include "Arduino.h"
#include <EEPROM.h>            // read and write from flash memory
#include <ESPAsyncWebServer.h>
#include "esp_task_wdt.h"      //to feed the task watchdog
#include <vector>
#include <algorithm>
#include "Credentials.h"  
#include <WiFi.h>
#include <WiFiMulti.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunBME280.h> //edit this library to change the i2c address if bme is not found
#include <SparkFun_SGP30_Arduino_Library.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>


//global vals
//hardware
bool bmeMounted = false;
bool sgpMounted = false;
bool dehumidiferState = false;
bool automaticDehumidifier = true;
bool automaticVpd = true;
bool heaterState = false;
int fanPower = 25;
int minpercentvalue = 25; //min power percentage required to make fan spin

//pinout
int dehumidifierControlPin = 13;
int fanControlPin = 12;
int heaterControlPin = 33;
int viberatorControlPin = 26;
int neopixelPin = 19;

//environ vals
float temp = 0;
float humidity = 0;
float co2 = 0;
float tvoc = 0;
float upperHumidityBound = 70.0f;
float lowerHumidityBound = 60.0f;
float targetVpd = 1.0f;
int sensorInterval = 2000;
char sensorjson[230];   //There is a max size u can send to MQTT broker

//time
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 3600;

//mqtt
long lastMsg = 0;
char msg[50];
int value = 0;

// setting PWM properties
const int freq = 25000;
const int fanPWMchannel = 0;
const int resolution = 12;

//global objects
WiFiMulti wifiMulti;
AsyncWebServer server(80);
Adafruit_NeoPixel pixels(1, neopixelPin, NEO_GRB + NEO_KHZ800);
BME280 bme280;
SGP30 sgp30;
TaskHandle_t longPWMTaskHandle = NULL;
TaskHandle_t LedAnimationTaskHandle = NULL;
TaskHandle_t freezewatchdogTaskHandle = NULL;
WiFiClientSecure espClient;
PubSubClient mqttclient(espClient);

int loopCounter = 0;
bool setupReceived = false;


const char start_html[] PROGMEM = R"rawliteral(<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Grow box</title>
  <script src='https://code.jquery.com/jquery-2.2.4.js'></script>
</head>
<body>
  <p>Dehumidifier settings</p>
  <input type='number' id='lower' placeholder='lower humidity bound'></input><br><button id='setLower'>Set lower</button><br>
  <br><input type='number' id='higher' placeholder='higher humidity bound'></input><br><button id ='setHigher'>Set higher</button>
  <p id="currentvals"></p>
  <script>
              function setLowerBound(){
                var oReq = new XMLHttpRequest();
                oReq.open("GET", "/lowBound?int="+document.getElementById("lower").value);
                oReq.send();
              }  

              function setHigherBound(){
                var oReq = new XMLHttpRequest();
                oReq.open("GET", "/highBound?int="+document.getElementById("higher").value);
                oReq.send();
              } 
              document.getElementById("setLower").onclick = function() {setLowerBound()};
              document.getElementById("setHigher").onclick = function() {setHigherBound()};


              function getEnviron(){
                      var oReq = new XMLHttpRequest();
                      oReq.addEventListener("load", environDisplay);
                      oReq.open("GET", "/environ");
                      oReq.send();
               }
              function environDisplay(){
                console.log(this.responseText)
                 document.getElementById('currentvals').innerHTML = this.responseText;
              }

              function sleep(milliseconds) {
                var start = new Date().getTime();
                for (var i = 0; i < 1e7; i++) {
                  if ((new Date().getTime() - start) > milliseconds){
                    break;
                  }
                }
              }

              sleep(2000);
              setInterval(getEnviron, 5000);
              getEnviron();
    </script>
  </body>  
</html>)rawliteral";

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
  //Wire.begin(13, 12);               //SDA orange, SCL purple      // Default is SDA 14, SCL 15
  bme280.setI2CAddress(0x76);

  if(bme280.beginI2C()){
    bmeMounted = true;
    Serial.println("bme280 found");
  } else {
    Serial.println("bme280 not found"); 
    i2cScan();
  }
  if(sgp30.begin()){
    sgpMounted = true;
    sgp30.initAirQuality();
    Serial.println("sgp30 found");
  } else {
    Serial.println("sgp30 not found");
    //i2cScan();
  }
}


void getSensorReadings(){
  //First 15 readings from SGP30 will be
  //CO2: 400 ppm  TVOC: 0 ppb as it warms up
  if(!bmeMounted){
    if(bme280.beginI2C()){
      bmeMounted = true;
    } 
  }
  if(!sgpMounted){
    if(sgp30.begin()){
      sgpMounted = true;
      sgp30.initAirQuality();
    }
  }


  if(!bmeMounted && !sgpMounted){return;}
  Serial.print("**");
  if(bmeMounted){
    humidity = bme280.readFloatHumidity();
    temp = bme280.readTempC();
    if(temp <0){
      temp = -1;
      humidity = -1;
      bmeMounted = false;
    }
    if(temp >=30){
      heaterState = false;
    }
  }

  if(sgpMounted){
    sgp30.measureAirQuality();
    co2 = sgp30.CO2;
    tvoc = sgp30.TVOC;
  }
}

float calcTargetHumdityForVpd(float targetVpd){     //humidity required to hit current temp

  float svp = 610.7 * (pow(10, (7.5*temp/(237.3+temp))));  
  float hu = ((((targetVpd * 1000) / svp) * 100) - 100) * -1 ;

  return hu;
}

float calcVpd(){

  float svp = 610.7 * (pow(10, (7.5*temp/(237.3+temp))));  
  float vpd = (((100 - humidity)/100)*svp)/1000; 

  return vpd;
}


void UpdateSensorJson(){
  StaticJsonDocument<200> doc;  
  doc["T"] = temp;
  doc["RH"] = humidity;
  doc["co2"] = co2;
  doc["tvoc"] = tvoc;
  doc["vpd"] = calcVpd();
  doc["targetVpd"] = targetVpd;
  doc["autoVpd"] = automaticVpd;
  doc["lowerBound"] = lowerHumidityBound;
  doc["upperBound"] = upperHumidityBound;
  doc["dehumidState"] = dehumidiferState;
  doc["autoDehumid"] = automaticDehumidifier;
  doc["fan"] = fanPower;
  doc["heaterState"] = heaterState;
  serializeJson(doc, sensorjson);
}


void pressDehumidifierButton(){
  pinMode(dehumidifierControlPin, OUTPUT);
  Serial.println("Switching dehumdifier");
  digitalWrite(dehumidifierControlPin, HIGH);  
  delay(500); 
  digitalWrite(dehumidifierControlPin, LOW); 
  dehumidiferState = !dehumidiferState;
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

void setFanPower(int power){ /// 0-100
  int powerVal = map(power, 0, 100, 0, 4095);
  esp_task_wdt_reset();
  ledcWrite(fanPWMchannel, powerVal);
}

void longPWMloop(void * parameter){
  Serial.println("started long PWM loop");
  int totalwidth = 50;
  int oncount = (fanPower - 1)/ 2 + 1;      //sweet trick to devide and get an int
  int count = 0;
  //Serial.println(oncount);
  for(;;){ // infinite loop
    if(count > totalwidth){
      count = 0;
    }
    if(count < oncount){
      //Serial.println("long PWM loop on");
      setFanPower(100);
    } else {
      //Serial.println("long PWM loop off");
      setFanPower(0);
    }

    count+=1;
    vTaskDelay(2000);
  }
}

void freezeWatchdog(void * parameter){
  int lastloopCounter=0;
  while(true){
    lastloopCounter = loopCounter;
    vTaskDelay(20000);
    if(loopCounter == lastloopCounter){
      Serial.println("Controller is frozen!!");
      ESP.restart();
    } else {
      Serial.println("watchdog is watching");
    }
  }
}


//neopixel animations
void rbgloop(void * parameter){
  int i = 0;
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
    if (i>2){
      i=0;
    }
    vTaskDelay(800);
  }
}

void mqttconnectloop(void * parameter){
  bool col = true;
  for(;;){ // infinite loop
    
    if (col){
        setPixelColor(pixels.Color(14, 218, 240));
      }else{
        setPixelColor(pixels.Color(7, 50, 224));
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
      mqttclient.subscribe("box/control/dehumidifier/auto");
      mqttclient.subscribe("box/control/dehumidifier/autoVpd");
      mqttclient.subscribe("box/control/dehumidifier/lower");
      mqttclient.subscribe("box/control/dehumidifier/upper");
      mqttclient.subscribe("box/control/dehumidifier/target");
      mqttclient.subscribe("box/control/dehumidifier/press");
      mqttclient.subscribe("box/control/exhaust");
      mqttclient.subscribe("box/control/heater");
      mqttclient.subscribe("box/control/vibe");
      mqttclient.subscribe("box/control/targetVpd");
      mqttclient.subscribe("box/environ");

    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttclient.state());
      Serial.println(" try again in 5 seconds");
      errcount++;
      if(errcount>1){
        ESP.restart();
      }
      // Wait 2 seconds before retrying
      delay(2000);
      
    }
  }
  
}

void vibrationPattern(int length){
  for (int i = 0; i < 4; i++) {
    digitalWrite(viberatorControlPin, HIGH); 
    delay(length);
    digitalWrite(viberatorControlPin, LOW); 
    delay(500);
  }
}

void mqttPublishSensorData(){ //TODO check if mqtt in creds
  if(!mqttclient.connected()){
    mqttreconnect();
  }

  UpdateSensorJson();
  // Serial.print("json updated");
  // Serial.println(sensorjson);
  bool success = mqttclient.publish("box/environ", sensorjson, true);
  if(success){
    Serial.print("->");
  } else {
    Serial.print("MQTT send failed with"); Serial.println(sensorjson);
  }
  //Serial.println("data published to MQTT server");
  
}



void mqttMessageReceived(char* topic, byte* message, unsigned int length) {
  esp_task_wdt_reset();
  //Serial.print("Message arrived on topic: ");
  //Serial.print(topic);
  //Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    //Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  //Serial.println();
  if (String(topic) == "box/environ" && setupReceived == false) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, messageTemp);
    JsonObject obj = doc.as<JsonObject>();
    long lastdehumidiferState = obj[String("dehumidState")];
    long autodehumidifer = obj[String("autoDehumid")];
    long autoVpd = obj[String("autoVpd")];
    int fan = obj[String("fan")];
    float lower = obj[String("lowerBound")];
    float upper = obj[String("upperBound")];
    int heater = obj[String("heaterState")];
    float tvpd = obj[String("targetVpd")];

    Serial.println("got last setup");
    Serial.print("target VPD ");
    Serial.println(tvpd);
    dehumidiferState = bool(lastdehumidiferState);
    automaticDehumidifier = bool(autodehumidifer);
    automaticVpd = bool(autoVpd);
    heaterState = bool(heater);
    fanPower = fan;
    lowerHumidityBound = lower;
    upperHumidityBound = upper;
    targetVpd = tvpd;

    setupReceived = true;
    mqttclient.unsubscribe("box/environ");
    return;
  }

  if (String(topic) == "box/control/exhaust") {
    int num = messageTemp.toInt();
    if(num>100){ num = 100;}
    if(num<0){num = 0;}
    fanPower = num;

    if(longPWMTaskHandle != NULL) {
            vTaskDelete(longPWMTaskHandle);
    }

    if(num< minpercentvalue){
        ///let the pwm loop task handle the fan
      xTaskCreate(
      longPWMloop,      // Function that should be called
      "longPWM",        // Name of the task (for debugging)
      1000,             // Stack size (bytes)
      &fanPower,             // Parameter to pass
      1,                // Task priority
      &longPWMTaskHandle);       // Task handle
      
    } else {
      setFanPower(num);
    }

    // char data[50];
    // snprintf_P(data, sizeof(data), PSTR("{\"fan\":%i}"), fanPower);
    // mqttclient.publish("box/environ", data);
    mqttPublishSensorData();
    return;
  }


  if (String(topic) == "box/control/dehumidifier/press") {
    pressDehumidifierButton();
    return;
  }

  if (String(topic) == "box/control/vibe") {
    int num = messageTemp.toInt();
    vibrationPattern(num);
    return;
  }

  if(String(topic) == "box/control/targetVpd"){
    float num = messageTemp.toFloat();
    if(num <= 2.0f && num >=0.4f){
      targetVpd = num;
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

    // char data[50];
    // snprintf_P(data, sizeof(data), PSTR("{\"autoDehumid\":%d}"), automaticDehumidifier);
    // mqttclient.publish("box/environ", data);
    mqttPublishSensorData();
    return;
  }

  if (String(topic) == "box/control/dehumidifier/autoVpd") {
    
    if(messageTemp == "on"){
      automaticVpd = true;
    }
    else if(messageTemp == "off"){
      automaticVpd = false;
    }

    char data[50];
    snprintf_P(data, sizeof(data), PSTR("{\"autoVpd\":%d}"), automaticVpd);
    mqttclient.publish("box/environ", data);
    return;
  }

  if (String(topic) == "box/control/dehumidifier/lower") {
    float num = messageTemp.toFloat();
    lowerHumidityBound = num;
    // char data[50];
    // snprintf_P(data, sizeof(data), PSTR("{\"lowerBound\":%f}"), lowerHumidityBound);
    // mqttclient.publish("box/environ", data);
    mqttPublishSensorData();
    return;
  }

  if (String(topic) == "box/control/dehumidifier/upper") {
    float num = messageTemp.toFloat();
    upperHumidityBound = num;
    // char data[50];
    // snprintf_P(data, sizeof(data), PSTR("{\"upperBound\":%f}"), upperHumidityBound);
    // mqttclient.publish("box/environ", data);
    mqttPublishSensorData();
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
    // char data[50];
    // snprintf_P(data, sizeof(data), PSTR("{\"heaterState\":%d}"), heaterState);
    // mqttclient.publish("box/environ", data);
    mqttPublishSensorData();
    return;
  }
}



void setup() {
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector 
  Serial.begin(115200);
  while (!Serial) { delay(10); } // Wait for serial console to open!
  
  Wire.begin();
  Wire.setClock(400000);

  if(LED){
    pixels.begin();
    pixels.setBrightness(30);
    startLedanimation(rbgloop);
  }

  
  ledcSetup(fanPWMchannel, freq, resolution);
  ledcAttachPin(fanControlPin, fanPWMchannel);

  pinMode(heaterControlPin, OUTPUT);
  digitalWrite(heaterControlPin, LOW); 
  
  //sensors
  initSensors();
 
  wifiMulti.addAP(SSIDNAME, WIFIPASS);    //Add additional networks here
  wifiMulti.run();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(2000);
  }

  Serial.println("");
  Serial.print("Connected to: ");
  Serial.println(SSIDNAME);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  //TODO check MQTT info is defined in creds
  Serial.print("MQTT URL ");
  Serial.println(MQTTURL);
  Serial.print("MQTT PORT ");
  Serial.println(MQTTPORT);
  mqttclient.setServer(MQTTURL, MQTTPORT);
  mqttclient.setCallback(mqttMessageReceived);

  startLedanimation(mqttconnectloop);
  mqttreconnect();
  mqttclient.publish("box/environ", "hi");
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
  
  xTaskCreate(freezeWatchdog, "watchdog", 1000, 0, 0, &freezewatchdogTaskHandle);
  flash3green();
}

void operateDehumidifier(){   //if humidity too high turn off the dehumifier and vice-versa
  if(bmeMounted !=(float)0 && humidity!=(float)0){
    Serial.print("O-");
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


void loop() {
  if(setupReceived == false){
    Serial.println("Waiting for a previous setup via mqtt");
    setPixelColor(pixels.Color(96, 43, 153));
    for(int i=0; i < 20; i++){    //lil loop to get previous setup
      mqttclient.loop();
      delay(500);
      if(setupReceived){break;}
    }
    setupReceived = true;
    setFanPower(fanPower);
    setPixelColor(pixels.Color(0, 0, 0));
  }


  mqttclient.loop();

  if(!mqttclient.connected()){
    mqttreconnect();
  }
  
  getSensorReadings();

  if(loopCounter%5 == 0){    //every sensorInterval * num seconds i%num
    if(automaticDehumidifier){
      if(automaticVpd){
        float hu  = calcTargetHumdityForVpd(targetVpd);
        //Serial.println(hu);
        lowerHumidityBound = hu - 1.0f;
        upperHumidityBound = hu + 1.0f;
      }
      operateDehumidifier();
    }
    
    mqttPublishSensorData();
  }


  delay(sensorInterval);
  loopCounter++;    //changing this value resets the freezewatchdog
  if(loopCounter > 100){
    loopCounter = 0;
  }

}