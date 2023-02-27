#include <globals.h>
#include <buffer.h>

#ifdef SCD4

    void initSensors(){
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
    }


    void getSensorReadings(){   //This can be done every 5 seconds
    if(SCD40Mounted){
        uint16_t error = scd4x.readMeasurement(co2, temp, humidity);
        // Serial.println(co2);
        // Serial.println(temp);
        // Serial.println(humidity);
        if(error || (co2 == -1 || temp == -1 || humidity == -1)){
        Serial.print("Error reading sensor values from SC40 ::");
        char errorMessage[256];
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
        }else{
        Serial.print("R ");
        Serial.print(humidity, 4);
        }
        humidityBufferWrite(humidity);

        float num = 0;
        int dupecount =0;
        for(int i=1; i<5; i++){
        int index = humidityBuffer.newest_index-i;
        if(index < 0){
            index = BUFFER_SIZE - index;
        }
        float c  = humidityBuffer.data[index];
        if(c == num){
            dupecount++;
        }
        num = c;
        }
     
     
        if(dupecount > 2){
        Serial.println("reinit sensors");
        initSensors();
        //scd4x.reinit();
        return;
        }

    }
    }

#endif