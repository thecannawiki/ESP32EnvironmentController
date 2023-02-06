
#include <math.h>

float calcTargetHumidityForVpd(float targetVpd, float temp){     //returns the humidity required to hit target VPD at current temp
  if(temp != -1){
    float svp = 610.7 * (pow(10, (7.5*temp/(237.3+temp))));  
    float hu = ((((targetVpd * 1000) / svp) * 100) - 100) * -1 ;
    return hu;
  }
  else{
    return -1;
  }
}


float calcVpd(float temp, float humidity){    //Calc vpd from air temp humidity

  float svp = 610.7 * (pow(10, (7.5*temp/(237.3+temp))));  
  float vpd = (((100 - humidity)/100)*svp)/1000; 

  return vpd;
}