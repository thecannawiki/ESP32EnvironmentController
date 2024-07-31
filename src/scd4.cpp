#include <globals.h>
#include <buffer.h>

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
            uint16_t error = scd4x.readMeasurement(co2, temp, humidity);
            // Serial.println(co2);
            // Serial.println(temp);
            // Serial.println(humidity);
            if(error || co2 == -1 || temp == -1 || humidity == -1){
                Serial.print("Error reading sensor values from SC40 ::");
                char errorMessage[256];
                errorToString(error, errorMessage, 256);
                Serial.println(errorMessage);
                success = false;
            } else {
                Serial.print("R ");
                Serial.print(humidity, 4);
                success = true;
            }
            humidityBufferWrite(humidity);  // write val to buffer whether it is new or not
            return success;
        }
        return false;
    }

#endif