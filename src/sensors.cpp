#include <globals.h>
#include <peripherals.h>
#include <buffer.h>
#include <scd4.h>
#include <bme.h>

int sensorMountFailCount = 0;

template<typename T>
void logln(T last) {
    Serial.println(last);
}

// Recursive case: print one arg, then recurse
template<typename T, typename... Args>
void logln(T first, Args... rest) {
    Serial.print(first);
    Serial.print(' ');
    logln(rest...);
}

void i2cScan(){
  byte error, address; //variable for error and I2C address
  Serial.println("Scanning...");

  int nDevices = 0;
  for (address = 1; address < 127; address++ ){

    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0){
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      switch(address){
        case 118:
          bmeMounted = true;
          Serial.println("bme280 found");
          break;
        case 98:
          SCD40Mounted = true;
          Serial.println("scd40 found");
          break;
      }

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0){
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
    if(SCD40Mounted){
      sensorPref = SCD4;
      logln("Sensor set to SCD40");
      return;
    }
    if(bmeMounted){
      sensorPref = BM280_SGP30;
      logln("Sensor set to BME280");
    }
  }
}

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

bool checkSensors(){

  if(sensorPref == SCD4){
    if(!SCD40Mounted){
      bool ok = initSensors();
      if(!ok){
        sensorMountFailCount +=1;
      } else {
        sensorMountFailCount = 0;
      }
      return ok;
    } else {
      return true;
    }

  } else if (sensorPref == BM280_SGP30)
  {
    if(!bmeMounted){
      bool ok = initSensors();
      if(!ok){
        sensorMountFailCount +=1;
      } else {
        sensorMountFailCount = 0;
      }
      return ok;
    } else {
      return true;
    }
  }
  
 

  if(sensorMountFailCount >= 8){
    Serial.println("Failed to connect sensors >4 times in a row. SAFETY MODE");
    setHeaterState(false);
    i2cScan();
    temp = -1;
    humidity = -1;
    fanPower = 30;
    fanChanged = true;
  }


  float num = 0;
  int dupecount = 0;
  for(int i=1; i<5; i++){
    int index = (humidityBuffer.newest_index - i + BUFFER_SIZE) % BUFFER_SIZE;
    float c  = humidityBuffer.data[index];
    if(c == num){
        dupecount++;
    }
    num = c;
  }

  if(dupecount > 4){
    setHeaterState(false);
    Serial.println("reinit sensors");
    initSensors();
  }

  return false;
}