#include <globals.h>

void setPixelColor(uint32_t color){
  pixels.setPixelColor(0, color);
  pixels.show();
}

void rbgloop(void * parameter){
  int i = 0;
  int totalLoopCounter = 0;
  for(;;){ // infinite loop
    
    switch (i){
      case 0:
        setPixelColor(pixels.Color(255,0,0));
        break;
      case 1:
        setPixelColor(pixels.Color(0,255,0));
        break;
      case 2:
        setPixelColor(pixels.Color(0,0,255));
        break;
    }

    i++;
    totalLoopCounter++;
    if (i>2){
      i=0;
    }
    vTaskDelay(800);
    if (totalLoopCounter > 7){
      setPixelColor(pixels.Color(255, 153, 51)); //orange
      vTaskDelay(1000);
      ESP.restart();
    }
  }
}

void mqttconnectloop(void * parameter){
  bool col = true;
  for(;;){ // infinite loop
    
    if (col){
        setPixelColor(pixels.Color(14, 218, 240));  //blue
      }else{
        setPixelColor(pixels.Color(7, 50, 224));    //different blue
    }

    col = !col;
    vTaskDelay(1000);
  }
}

void flash3green(){
  int delayTime = 90;
  int onTime = 120;
  setPixelColor(pixels.Color(0, 0, 0));
  delay(300);
  setPixelColor(pixels.Color(0,255,0));
  delay(onTime);
  setPixelColor(pixels.Color(0, 0, 0));
  delay(delayTime);
  setPixelColor(pixels.Color(0,255,0));
  delay(onTime);
  setPixelColor(pixels.Color(0, 0, 0));
  delay(delayTime);
  setPixelColor(pixels.Color(0,255,0));
  delay(onTime);
  setPixelColor(pixels.Color(0, 0, 0));
}


void startLedanimation(void (*func)(void *)){
  if(LedAnimationTaskHandle != NULL){
    vTaskDelete(LedAnimationTaskHandle);
  }
  xTaskCreate(func, "loop", 1000, 0, 0, &LedAnimationTaskHandle);  
}