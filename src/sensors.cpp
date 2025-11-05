#include <globals.h>
#include <buffer.h>
#include <scd4.h>
#include <bme.h>

    bool initSensors(){
        if(sensorPref == SCD4){
          return initScd40();

        }
        else if(sensorPref == BM280_SGP30){
            return initBmeSgp();
        }
        return false;
    }


    bool getSensorReadings(){   //This can be done every 5 seconds
        if(sensorPref == SCD4){
            return readScd40();
        
        }
        else if(sensorPref == BM280_SGP30){
            return readBmeSgp();
        }

        return false;
    }
