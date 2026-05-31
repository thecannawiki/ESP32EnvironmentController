#include <globals.h>
#include <buffer.h>
#include <Wire.h>

int sensorReadCount = 0;

bool initBmeSgp(){
    bme280.setI2CAddress(0x76);
    bme280.setTempOverSample(1);      // was probably 16 — biggest time saving
    bme280.setPressureOverSample(1);
    bme280.setHumidityOverSample(1);
    bme280.setStandbyTime(0);          // minimum standby between readings

    if(bme280.beginI2C(Wire)){
        bmeMounted = true;
    } else {
        Serial.println("bme280 not found"); 
        bmeMounted = false;
        //i2cScan();
    }
    if(sgp30.begin(Wire)){
        sgpMounted = true;
        sgp30.initAirQuality();
        Serial.println("sgp30 found");
    } else {
        Serial.println("sgp30 not found");
        sgpMounted = false;
        //i2cScan();
    }
    return bmeMounted; //Only temp/humidity is essential
}

bool readBmeSgp(){
    //First 15 readings from SGP30 will be
    //CO2: 400 ppm  TVOC: 0 ppb as it warms up
    bool success=false;
    if(!bmeMounted){
        if(bme280.beginI2C(Wire)){
            bmeMounted = true;
        } 
    }
    if(!sgpMounted){
        if(sgp30.begin(Wire)){
            sgpMounted = true;
            sgp30.initAirQuality();
        }
    }

    if(!bmeMounted && !sgpMounted){return false;}
    
    if(bmeMounted){
        float h = bme280.readFloatHumidity();
        float t = bme280.readTempC();
        // vTaskDelay(10);
        if(t <0 || (sensorReadCount > 5 && t > 2*tempBuffer.avgOfLastN(3))){ 
            temp = -1;
            humidity = -1;
            bmeMounted = false;
            success = false;
        } else {
            sensorReadCount++;
            Serial.print("* ");
            success = true;

            humidityBuffer.write(h);
            tempBuffer.write(t);

            humidity = humidityBuffer.avgOfLastN(3);
            temp = tempBuffer.avgOfLastN(3);
        }
    }

    if(sgpMounted){
        sgp30.measureAirQuality();
        co2 = sgp30.CO2;
        tvoc = sgp30.TVOC;
        Serial.print("÷ ");
    }

    return success;
}
