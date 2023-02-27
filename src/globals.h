
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
    extern SensirionI2CScd4x scd4x;
    extern bool SCD40Mounted;
#endif

#ifdef BME
    #include <SparkFunBME280.h> //edit this library to change the i2c address if bme is not found
    #include <SparkFun_SGP30_Arduino_Library.h>
    extern BME280 bme280;
    extern SGP30 sgp30;
    extern bool bmeMounted;
    extern bool sgpMounted;
#endif

//global vals
//hardware

extern bool dehumidiferState;
extern bool automaticDehumidifier;
extern bool automaticVpd;
extern bool heaterState;
extern float fanPower;
extern float minpercentvalue; //min power percentage required to make fan spin 

//pinout
extern int dehumidifierControlPin;
extern int fanControlPin;
extern int heaterControlPin;
extern int stirrerControlPin;
extern int neopixelPin;

//environ vals
extern float temp;
extern float targetHumidity;
extern float humidity;
extern uint16_t co2;
extern float tvoc;
extern float upperHumidityBound;
extern float lowerHumidityBound;
extern float targetVpd;
extern int sensorInterval;
extern char sensorjson[240];   //There is a max size u can send to MQTT broker


//PID
extern float P;
extern float I;
extern float D;

//time
extern const char* ntpServer;
extern long  gmtOffset_sec;
extern int   daylightOffset_sec;

//mqtt
extern long lastMsg;
extern char msg[50];
extern int value;

// setting PWM properties
extern int freq;
extern int fanPWMchannel;
extern int resolution;
extern int maxPWMval;
extern bool fanChanged;

//global objects
extern WiFiMulti wifiMulti;
extern AsyncWebServer server;
extern Adafruit_NeoPixel pixels;

extern TaskHandle_t longPWMTaskHandle;
extern TaskHandle_t LedAnimationTaskHandle;
extern TaskHandle_t freezewatchdogTaskHandle;
extern TaskHandle_t mainLoopTaskHandle;
extern TaskHandle_t mqttTaskHandle;
extern WiFiClientSecure espClient;
extern PubSubClient mqttclient;
extern Preferences preferences;

//runtime 
extern int loopCounter;
extern bool setupReceived;