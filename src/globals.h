
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
#include "buffer.h"
#include <SensirionI2CScd4x.h>
#include <SparkFunBME280.h> //edit this library to change the i2c address if bme is not found
#include <SparkFun_SGP30_Arduino_Library.h>

#ifndef GLOBALS_H
#define GLOBALS_H

enum Sensor {BM280_SGP30, SCD4};

extern SensirionI2CScd4x scd4x;
extern bool SCD40Mounted;
extern BME280 bme280;
extern SGP30 sgp30;
extern bool bmeMounted;
extern bool sgpMounted;
extern Sensor sensorPref;
//global vals
//hardware


extern bool dehumidiferState;
extern bool humidifierState;
extern bool automaticDehumidifier;
extern bool automaticHumidifier;
extern bool automaticFanVpd;
extern bool dehumidifierPrimaryMode;
extern bool dehumidifierForTemp;
extern bool heaterState;
extern int heaterPower;
extern bool heaterTempMode;
extern bool autoHeater;
extern int waterSensor1State;
extern int waterSensor2State;
extern bool pumpState;
extern float fanPower;
extern float lastPowerVal;
extern float softMaxFan;
extern float softMinFan;
extern float minpercentvalue; //min power percentage required to make fan spin 
extern bool lockHVAC;    //prevents changes to any HVAC equiptment

//settings
extern bool vpdMode;
extern bool setupMode;
extern char deviceName[40];
extern char MQTTPUBLISHTOPIC[50];
extern char MQTTCONTROLTOPIC[50];


//pinout
extern int dehumidifierControlPin;
extern int humidifierControlPin;
extern int fanControlPin;
extern int pumpControlPin;
extern int heaterControlPin;
extern int stirrerControlPin;
extern unsigned long pumpEnd;
extern unsigned long pumpStart;
extern unsigned long heaterStart;
extern unsigned long heaterEnd;
extern int neopixelPin;
extern int waterSensor1Pin;
// extern int waterSensor2Pin;
extern int setupModePin;

//environ vals
extern float temp;
extern float targetHumidity;
extern float targetTemperature;
extern float ventTemp;
extern float humidity;
extern uint16_t co2;
extern float tvoc;
extern float upperHumidityBound;
extern float lowerHumidityBound;
extern float targetVpd;
constexpr size_t SENSORJSON_SIZE = 420; //There is a max size u can send to MQTT broker
extern char sensorjson[SENSORJSON_SIZE];  
extern int w1maxWaterSensorVal;

extern Buffer humidityBuffer;
extern Buffer tempBuffer;
extern Buffer errorBuffer;
extern Buffer w1Buffer;
extern Buffer fanBuffer;


//PID
extern float P;
extern float I;
extern float D;

//time
extern const char* ntpServer;
extern long  gmtOffset_sec;
extern int   daylightOffset_sec;
extern struct tm timeinfo;
extern time_t timeNow;

//mqtt
extern long lastMsg;
extern char msg[50];
extern int value;

// setting PWM properties
extern int freq;
extern int heaterFreq;
extern int fanPWMchannel;
extern int heaterPWMchannel;
extern int resolution;
extern int maxPWMval;
extern bool fanChanged;
extern float fanSoftMaxPWM;
extern float heaterSoftMaxPWM;


//global objects
extern WiFiMulti wifiMulti;
extern AsyncWebServer server;
extern Adafruit_NeoPixel pixels;

extern TaskHandle_t longPWMTaskHandle;
extern TaskHandle_t LedAnimationTaskHandle;
extern TaskHandle_t TransTestTaskHandle;
extern TaskHandle_t freezewatchdogTaskHandle;
extern TaskHandle_t mainLoopTaskHandle;
extern TaskHandle_t mqttTaskHandle;
extern WiFiClientSecure espClient;
extern PubSubClient mqttclient;
extern Preferences preferences;

//runtime 
extern int loopCounter;
extern bool setupReceived;
extern int transTestTime;


#endif