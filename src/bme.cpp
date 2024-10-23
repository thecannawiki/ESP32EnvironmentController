#include <globals.h>
#include <buffer.h>

#ifdef BME
    bool initSensors(){
        //Wire.begin(13, 12);               //SDA orange, SCL purple      // Default is SDA 14, SCL 15
        bme280.setI2CAddress(0x76);

        if(bme280.beginI2C()){
            bmeMounted = true;
            Serial.println("bme280 found");
        } else {
            Serial.println("bme280 not found"); 
            bmeMounted = false;
            //i2cScan();
        }
        if(sgp30.begin()){
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

    bool getSensorReadings(){
    //First 15 readings from SGP30 will be
    //CO2: 400 ppm  TVOC: 0 ppb as it warms up
        bool success=false;
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


        if(!bmeMounted && !sgpMounted){return false;}
        
        if(bmeMounted){
            float h = bme280.readFloatHumidity();
            float t = bme280.readTempC();
            if(t <0){
                temp = -1;
                humidity = -1;
                bmeMounted = false;
                success = false;
            } else {
                Serial.print("* ");
                success = true;

                //BME requires a filter to prevent crazy spikes
                // float last_humidity = humidity;
                // humidity = (last_humidity + h) / 2;

                humidityBuffer.write(h);
                tempBuffer.write(t);

                Serial.println("hu");
                humidityBuffer.printData();

                Serial.println("temp");
                tempBuffer.printData();
                humidity = humidityBuffer.avgOfLastN(3);
                temp = humidityBuffer.avgOfLastN(3);

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

#endif