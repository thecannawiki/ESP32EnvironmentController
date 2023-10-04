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