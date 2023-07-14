#include <globals.h>


void saveToNVM(){
  preferences.end();
  Serial.print("§ ");
  preferences.begin("controller", false);
}  