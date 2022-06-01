# ESP32EnvironmentController
An environment (Temperature, humidity, Co2) controller for ESP32

Feature list:
* Post live data to MQTT server
* Control fan speed and dehumidifer state remotely (Can also control other appliances)
* Compatibility with Node-red dashboard to display graphs of temperature, humidity, VPD, co2 levels and state of appliances being controlled.

Part list:
All parts are needed for full functionality but most are optional
* ESP32 board
* BME280 or BME6 temp and humidity sensor
* SGP30 Co2 sensor
* Neopixel LED (optional for status indicator)
* Resistors and NPN transistors, relays (For controlling fan)


Build instructions:
1. Download repo and open in platformio
2. Register on MQHive for a cloud MQTT Broker
3. Fill out Crendentials.h with credentials for wifi network and MQTT broker
4. Set COM port of your ESP32 in platformio.ini and flash the firmware
5. Install Node-red on a computer (or phone or raspberry pi)
6. In Node-red import the dashboard.json flow file in this repo. Configure MQTT broker login details
7. Connect sensors to the ESP32 and power on. You should see the ESP authenticate with your MQTT server via Serial monitor output


Peripheral connections:
These are the default pins to connect the BME and SGP30 sensors as well as any other devices to be controller
* The BME280 and SGP30 are both I2C devices that require 3.3v, GND and pin 21 for SDA and pin 22 for SCL
* An external fan be controlled via a switching transistor and pin 12 (Tested at 12v)
* A dehumidifier can be attached via a switching transistor on pin 13. Used with a transistor to press a momemary button on my dehumidifer. May not work with all dehumidifiers with modification.