#include <globals.h>
#include <buffer.h>

int sensorDupCount = 0;

#ifdef SCD4

    bool initSensors(){
        scd4x.begin(Wire);
        uint16_t error = scd4x.stopPeriodicMeasurement();
        if(error){
            Serial.println("failed to stop SCD40");
        }
        error = scd4x.startPeriodicMeasurement();
        
        if (error) {
            Serial.print("Error trying to execute startPeriodicMeasurement(): ");
            Serial.println(error);
            SCD40Mounted = false;
        } else {
            Serial.println("SCD-40 connected");
            SCD40Mounted = true;
        }
        return SCD40Mounted;
    }


    bool getSensorReadings(){   //This can be done every 5 seconds
        bool success = false;
        if(SCD40Mounted){
            float hu = 0.0f;
            float t = 0.0f;
            uint16_t error = scd4x.readMeasurement(co2, t, hu);
            // Serial.println(co2);
            // Serial.println(temp);
            // Serial.println(humidity);
            if(error || co2 == -1 || t == -1 || hu == -1){
                Serial.print("Error reading sensor values from SC40 ::");
                char errorMessage[256];
                errorToString(error, errorMessage, 256);
                Serial.println(errorMessage);
                
                Serial.println("hu fail val");
                int prevIndex =  (humidityBuffer.newest_index - 1 + BUFFER_SIZE) % BUFFER_SIZE;
                float hufailval = humidityBuffer.data[prevIndex];
                Serial.println(hufailval);
                humidityBuffer.write(hufailval);

                Serial.println("temp fail val");
                prevIndex =  (tempBuffer.newest_index - 1 + BUFFER_SIZE) % BUFFER_SIZE;
                float tempfailval = tempBuffer.data[prevIndex];
                Serial.println(tempfailval);

                tempBuffer.write(tempfailval);

            } else {
                success = true;
                humidityBuffer.write(hu);  
                tempBuffer.write(t);
            }
            
            
            humidityBuffer.printData();

            humidity = humidityBuffer.avgOfLastN(2);
            temp = tempBuffer.avgOfLastN(2);

            if(success){
                Serial.print("R ");
                Serial.print(humidity, 4);
                return true;
            }
        }
        return false;
    }

#endif