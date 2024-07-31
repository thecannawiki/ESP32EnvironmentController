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

#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <Preferences.h>


#ifdef SCD4
    #include <SensirionI2CScd4x.h>
    SensirionI2CScd4x scd4x;
    bool SCD40Mounted = false;
#endif

#ifdef BME
    #include <SparkFunBME280.h> //edit this library to change the i2c address if bme is not found
    #include <SparkFun_SGP30_Arduino_Library.h>
    BME280 bme280;
    SGP30 sgp30;
    bool bmeMounted = false;
    bool sgpMounted = false;
#endif


//global vals
//hardware
bool dehumidiferState = false;
bool automaticDehumidifier = true;
bool automaticFanVpd = true;
bool dehumidifierPrimaryMode = false; // Primary mode means dehumidifier/humidifier will operate on bounds around target humidity. When false (secondary mode) it will control based on fan power usage
bool dehumidifierForTemp = false; // The dehumidifier  will operate on bounds around target temperature. This takes precedence over dehumidifier secondary mode when enabled
bool heaterState = false;
bool pumpState = false;
float fanPower = 30.0f;        //float to 1dp between 0 and 100
float lastPowerVal = 0;        //for fan ramp up control
float minpercentvalue = 0.0f; //min power percentage required to make fan spin e.g 0.25 for 25%
float softMaxFan = 100.0f; //soft limit on fan percentage
bool lockHVAC = false;
unsigned long pumpEnd = 0;
unsigned long pumpStart = 0;
int waterSensor1State = 0;
int waterSensor2State = 0;

//pinout
int dehumidifierControlPin = 13;
int fanControlPin = 12;
int pumpControlPin = 33;
int waterSensor1Pin= 34;
int waterSensor2Pin= 35;
int heaterControlPin = 26;
int neopixelPin = 19;

//environ vals
float temp = -1;
float humidity = -1;
float targetHumidity = 60;  //Sensible default
float targetTemperature = 25;
float ventTemp = 33;
uint16_t co2 = -1;
float tvoc = 0;
float upperHumidityBound = 60.0f;
float lowerHumidityBound = 40.0f;
float targetVpd = 1.0f;
char sensorjson[240];   //There is a max size u can send to MQTT broker


//PID
float P = 0.1;
float I = 0.1;
float D = 0.1;


//time
const char* ntpServer = "pool.ntp.org";
long gmtOffset_sec = 0;
int daylightOffset_sec = 3600;

//mqtt
long lastMsg = 0;
char msg[50];
int value = 0;

// setting PWM properties
int freq = 200000;
int fanPWMchannel = 0;
int resolution = 10;
int maxPWMval = 1023;
bool fanChanged = false;
float softMaxPWM = (MAXFANCONTROLLEROUTPUT / 100.0) * maxPWMval;

//global objects
WiFiMulti wifiMulti;
AsyncWebServer server(80);
Adafruit_NeoPixel pixels(1, neopixelPin, NEO_GRB + NEO_KHZ800);

TaskHandle_t longPWMTaskHandle = NULL;
TaskHandle_t LedAnimationTaskHandle = NULL;
TaskHandle_t freezewatchdogTaskHandle = NULL;
TaskHandle_t mainLoopTaskHandle = NULL;
TaskHandle_t mqttTaskHandle = NULL;
WiFiClientSecure espClient;
PubSubClient mqttclient(espClient);
Preferences preferences;

//runtime 
int loopCounter = 0;
bool setupReceived = false;