#include <globals.h>


void saveToNVM(){
  vTaskDelay(10); // yield b4 this
  preferences.end();
  Serial.print("§ ");
  preferences.begin("controller", false);
}  