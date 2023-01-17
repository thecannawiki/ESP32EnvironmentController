
#include <math.h>

float calcTargetHumidityForVpd(float targetVpd, float temp, float humidity){     //humidity required to hit target VPD at current temp
  if(temp != -1 && humidity != -1){
    float svp = 610.7 * (pow(10, (7.5*temp/(237.3+temp))));  
    float hu = ((((targetVpd * 1000) / svp) * 100) - 100) * -1 ;
    return hu;
  }
  else{
    return -1;
  }
}