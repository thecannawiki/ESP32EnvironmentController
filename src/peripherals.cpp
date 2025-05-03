#include <globals.h>

void pressDehumidifierButton(){
  pinMode(dehumidifierControlPin, OUTPUT);
  Serial.println("Switching dehumdifier");
  digitalWrite(dehumidifierControlPin, HIGH);  
  delay(500); 
  digitalWrite(dehumidifierControlPin, LOW); 
  dehumidiferState = !dehumidiferState;
  preferences.putBool("dehumidifier", dehumidiferState);
}

void setDehumidiferState(bool state){
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
  pinMode(humidifierControlPin, OUTPUT);
  
  if(state){
    digitalWrite(humidifierControlPin, HIGH);  
    Serial.println("turning humidifier ON");
  } else {
    digitalWrite(humidifierControlPin, LOW);
    Serial.println("turning humidifier OFF");
  }
  humidifierState = state;
  preferences.putBool("humidifier", humidifierState);
}

// TODO start a task when heater turns on to monitor the difference (use temp buffer), if it is different to expected kill heater
void setHeaterState(bool state){
  if(state){
    digitalWrite(heaterControlPin, HIGH);  
    Serial.println("turning heater ON");
  } else {
    digitalWrite(heaterControlPin, LOW);
    Serial.println("turning heater OFF");
  }

  heaterState = state;
  preferences.putBool("headerState", heaterState);
}

void startHeater(){
  setHeaterState(true);
  time(&timeNow);
  heaterEnd = timeNow + 10*60;
}