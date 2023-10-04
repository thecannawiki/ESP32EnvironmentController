
#include <math.h>
#include <Arduino.h>

// https://hmco.enpc.fr/UNIT_ComplexSystems/~gen/1_Introduction/1_Introduction_web.publi/web/co/module_Introduction_40.html
float calcSvp(float temp){  //takes temp in degrees celsius, returns saturation vapour pressure in Pascals
  return 610.7 * (pow(10, (7.5*temp/(237.3+temp))));
}

float calcTargetHumidityForVpd(float targetVpd, float temp){     //returns the humidity required to hit target VPD at current temp
  if(temp != -1){
    float svp = calcSvp(temp);  
    float hu = ((((targetVpd * 1000) / svp) * 100) - 100) * -1 ;
    return hu;
  }
  else{
    return -1;
  }
}



float calcVpd(float temp, float humidity){    //Calc vpd from air temp humidity

  float svp = calcSvp(temp);  
  float vpd = (((100 - humidity)/100)*svp)/1000; 

  return vpd;
}

double calculationAbsHum(float t, float h)
{
double temp;
temp = pow(2.718281828, (17.67 * t) / (t + 243.5));
return (6.112 * temp * h * 2.1674) / (273.15 + t);
}

float calcGramsOfWaterInAir(float temp, float humidity){    /// grams of water per cubic meter
  //There are a few different equations used with varying constant to calculate this.
  // Sensor accuracy for the SCD40 is 6%RH roughly that translates to ~3g/m^3

  Serial.println("temp");
  Serial.println(temp);

  Serial.println("humidity");
  Serial.println(humidity);

  float svp = calcSvp(temp);  
  Serial.println("svp");
  Serial.println(svp);

  // double pascalToAtmMultiplier = 0.00000986923;
  float vp = humidity * svp * 100; //vapour pressure = humidity * saturation vapour pressure


  Serial.println("vp");
  Serial.println(vp);
  // float mols = (vp * volume) / (0.0821 * temp + 273); //rearranged ideal gas equation n= PV/RT

  // float massOfWater = mols * 18; //mols * molar mass of water
  // float massOfWater = mols * 18; //mols * molar mass of water

  //AH = (RH × P)/(461.5 J/(kg⋅K) × T × 100)
  float massOfWater = (humidity * svp) /(461.5 * temp+273.15 * 100);
  // Serial.print("mass 1 ");
  // Serial.println(massOfWater);



  // double sp = 6.112 * pow(EULER, (17.67 * temp)/(temp+243.5));
  double P = svp * (humidity / 100);
  float mols = P / (8.2 * temp);

  float massOfWater2 = mols * 18.02;

  float massOfWater3 = humidity * 6.112 * 2.1674 * pow(EULER, ((temp* 17.67)/(temp+243.5))/(temp+273.15));

  double massOfWater4 = calculationAbsHum(temp, humidity);  // https://github.com/finitespace/BME280/issues/25


  //Calculating AH. Also try this one
  //http://www.noveldevices.co.uk/rp-about-humidity

   Serial.print("mass 1 ");
  Serial.println(massOfWater);

   Serial.print("mass 2 ");
  Serial.println(massOfWater2);
   Serial.print("mass 3 ");
  Serial.println(massOfWater3);



  Serial.print("mass 4 current");
  Serial.println(massOfWater4);

  return massOfWater4;
}



///Absolute humidity notes
//https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/