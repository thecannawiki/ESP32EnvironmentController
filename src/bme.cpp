#include <globals.h>
#include <buffer.h>

#ifdef BME
    void initSensors(){
    //Wire.begin(13, 12);               //SDA orange, SCL purple      // Default is SDA 14, SCL 15
    bme280.setI2CAddress(0x76);

    if(bme280.beginI2C()){
        bmeMounted = true;
        Serial.println("bme280 found");
    } else {
        Serial.println("bme280 not found"); 
        //i2cScan();
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
    
    if(bmeMounted){
        humidity = bme280.readFloatHumidity();
        temp = bme280.readTempC();
        if(temp <0){
        temp = -1;
        humidity = -1;
        bmeMounted = false;
        }else{
        Serial.print("* ");

        humidityBufferWrite(humidity);
        }
        if(temp >=30){
        heaterState = false;
        }
    }

    if(sgpMounted){
        sgp30.measureAirQuality();
        co2 = sgp30.CO2;
        tvoc = sgp30.TVOC;
        Serial.print("÷ ");
    }
    }

#endif