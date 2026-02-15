#include <globals.h>
#include <buffer.h>

void pressDehumidifierButton(){
  pinMode(dehumidifierControlPin, OUTPUT);
  Serial.println("Switching dehumidifier");
  digitalWrite(dehumidifierControlPin, HIGH);  
  delay(500); 
  digitalWrite(dehumidifierControlPin, LOW); 
  dehumidiferState = !dehumidiferState;
  preferences.putBool("dehumidifier", dehumidiferState);
}

void setDehumidifierState(bool state){
  pinMode(dehumidifierControlPin, OUTPUT);
  
  if(state){
    digitalWrite(dehumidifierControlPin, HIGH);  
    Serial.println("turning dehumidifier ON");
  } else {
    digitalWrite(dehumidifierControlPin, LOW);
    Serial.println("turning dehumidifier OFF");
  }
  dehumidiferState = state;
  preferences.putBool("dehumidifier", dehumidiferState);
}

void setHumidifierState(bool state){
  humidifierState = state;
  if(state){
    digitalWrite(humidifierControlPin, HIGH);  
    Serial.println("turning humidifier ON");
  } else {
    digitalWrite(humidifierControlPin, LOW);
    Serial.println("turning humidifier OFF");
  }
  
  preferences.putBool("humidifier", humidifierState);
}

// TODO start a task when heater turns on to monitor the difference (use temp buffer), if it is different to expected kill heater
void setHeaterState(bool state){
  if(state){
    // digitalWrite(heaterControlPin, HIGH);  
    heaterPower = heaterMaxPower;
    float p = heaterMaxPower / 100.0f;
    int power =  p * maxPWMval;

    ledcWrite(heaterPWMchannel, power);
    Serial.println("turning heater ON");
  } else {
    // digitalWrite(heaterControlPin, LOW);
    heaterPower = 0;
    ledcWrite(heaterPWMchannel, 0);
    Serial.println("turning heater OFF");
  }

  // heaterState = state;
  // preferences.putBool("headerState", heaterState);
}

void updateHeaterPower(){
  if(heaterPower > heaterMaxPower){ heaterPower=heaterMaxPower;}
  float p = heaterPower / 100.0f;
  int power =  p * maxPWMval;
  fanChanged = true; //Trigger fan update incase fan needs to be turned down due to high heater
  ledcWrite(heaterPWMchannel, power);
}


float PIDDTerm(int lookback_length){

  int lookback_index = humidityBuffer.newest_index-lookback_length;
  int cindex = humidityBuffer.newest_index-1;
  if(lookback_index < 0){
    lookback_index = humidityBuffer.size() + lookback_index;
  }
  if(cindex < 0){
    cindex = humidityBuffer.size() + cindex;
  }
  float old = humidityBuffer.data[lookback_index];
  float current = humidityBuffer.data[cindex];
  if(old ==-1.0f || old == 0.0f || current ==-1.0f || current== 0.0f){
    return 0;
  }

  double diff = ( current - old) / lookback_length;

  return diff;
}