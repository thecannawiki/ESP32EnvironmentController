# ESP32EnvironmentController
An environment (Temperature, humidity, Co2) controller for ESP32

![Node-red interface](https://github.com/user-attachments/assets/01f73acd-4df4-4e7e-9391-60e350c5a6d6)

## Feature list:
* Automatic fan and heater control to meet target VPD
* Post live data to MQTT server
* Sensor data retrievable via HTTP
* Using a cloud MQTT Broker makes the controller accessible from anywhere you have an internet connection
* Compatibility with Node-red dashboard to display graphs of temperature, humidity, VPD, co2 levels and state of appliances being controlled.
* All data is saved to a file on the Node-red server and graphs can be generated by using the DataPlotter.py script
* Automatic watering based on hourly schedule

## Part list:
Only the ESP32 and temp/humidity sensor are strictly required
* ESP32 board
* BME280 or SCD40 temp and humidity sensor
* Motor driver board, ideally H-bridge mosfet based driver or TB6612FNG for smaller motors
* SGP30 Co2 sensor (For Co2 reading when using the BME280. SCD40 is recommended)
* Neopixel LED (optional for status indicator)
* Relay for heater control


## Build instructions:
1. Register on MQHive for a cloud MQTT Broker or setup a MQTT broker locally.
2. Download this repo and open in Vscode. You will need the Platformio extension installed.
3. Fill out Credentials.h with credentials for your wifi network and MQTT broker.
4. Connect your ESP32 to the computer via usb
5. Set the COM port of your ESP32 in `platformio.ini` file and flash the firmware.

Once flashing is complete you should see the Esp32 connect to Wifi and authenticate with your MQTT broker via the serial monitor output.

6. Install Node-red on a computer (or phone or raspberry pi)
7. In Node-red import the dashboard.json (or Mobile.json on mobile) flow file in this repo.
8. In Node-red configure MQTT broker login details you created in step 2.
9. Connect sensors to the ESP32 (see below), and connect the esp to power either via the pins or USB port (Requires 5v 2A). Unfortunately, most PC usb ports only provide 1A so you will need a usb plug adapter or power supply.

## MQTT Topics
The Topic the grow controller will publish sensor data to is specified in credentials.h as MQTTPUBLISHTOPIC.
The controller can be instructed by sending messages to the MQTTCONTROLTOPIC + the endpoint
e.g MQTTCONTROLTOPIC + "dehumidifier/auto"
The following endpoints are supported:
/dehumidifier/autoVpd
/dehumidifier/auto
/dehumidifier/lower
/dehumidifier/upper
/dehumidifier/target
/dehumidifier/press
/exhaust
/targetVpd


## Sensor connections:
The BME280, SGP30, SCD40 and are all I2C devices that require the following pins on the esp32 `3.3v`, `GND`, `pin 21 for SDA` and `pin 22 for SCL`
When using the BME280 and SGP30 in conjunction you will need a splitter cable

## Peripheral connections:
* An external fan be controlled via a motor driver connected to pin 12. Be sure to use the appropriate driver for your fan size
* A dehumidifier can be attached via a relay or mosfet on pin 13
* A Neopixel LED can be connected to 5v and Pin 16 where it will act as a status indicator. This mostly for development and debug purposes
* waterSensor1 pin 34;
* waterSensor2 pin 4;

## User manual

### Control modes:
There are a number of switches that effect how the controller behaves
* Autovpdfan (On/Off) - The fan is controlled automatically to reach a target VPD
* Autodehumidifer (On/Off) - The dehumidifier is automatically controlled to maintain target VPD

* Primary dehumidifier mode - Automatically control dehumidifier to maintain VPD
* Secondary dehumidifier mode - dehumidifier is controlled based on fan power. turn on when over 90%, turn off when under 50%

