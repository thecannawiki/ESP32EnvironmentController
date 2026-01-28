#include <globals.h>
#include <buffer.h>
#include <Wire.h>

int sensorDupCount = 0;


bool initScd40(){
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
        Serial.println("SCD-40 initalized");
        SCD40Mounted = true;
    }
    return SCD40Mounted;
}


bool readScd40(){   //This can be done every 5 seconds
    bool success = false;
    if(SCD40Mounted){
        uint16_t data_ready = 0;
        uint16_t error = scd4x.getDataReadyStatus(data_ready);

        if (error) {
            Serial.println("Error checking data ready flag");
        } else
        if(data_ready == 0){
            Serial.println("SCD40 not ready");
            return false;
        }

        float hu = 0.0f;
        float t = 0.0f;
        error = scd4x.readMeasurement(co2, t, hu);
        // Serial.println(co2);
        // Serial.println(temp);
        // Serial.println(humidity);
        if(error || co2 == -1 || t == -1 || hu == -1){
            Serial.print("Error reading sensor values from SC40 :: ");
            char errorMessage[256];
            errorToString(error, errorMessage, 256);
            Serial.println(errorMessage);

            int prevIndex =  (humidityBuffer.newest_index - 1 + humidityBuffer.size()) % humidityBuffer.size();
            float hufailval = humidityBuffer.data[prevIndex];
            humidityBuffer.write(hufailval);

            prevIndex = (tempBuffer.newest_index - 1 + tempBuffer.size()) % tempBuffer.size();
            float tempfailval = tempBuffer.data[prevIndex];
            
            tempBuffer.write(tempfailval);

        } else {
            success = true;
            humidityBuffer.write(hu);  
            tempBuffer.write(t);
        }

        //humidityBuffer.printData();
        
        humidity = humidityBuffer.avgOfLastN(2);
        temp = tempBuffer.avgOfLastN(2);
        Serial.println(humidity);
        
        if(t == 0.0f && tempBuffer.avgOfLastN(8) == 0.0f){ // First read on the scd40 can be 0
            return false;
        }

        return success;
    }
    return false;
}
