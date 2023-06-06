#include "Arduino_BHY2.h"

#define THRESHOLD_GAS 500
#define THRESHOLD_TEMP 0.1
#define THRESHOLD_PRESSURE 0.02

Sensor pressure(SENSOR_ID_BARO);
Sensor step(SENSOR_ID_STD);
Sensor mag(SENSOR_ID_MAG);
Sensor airq(SENSOR_ID_HUM);
Sensor airbsec(SENSOR_ID_BSEC);
Sensor gas(SENSOR_ID_GAS);
Sensor temp(SENSOR_ID_TEMP);



void setup() {
  Serial.begin(115200);
  BHY2.begin();

  pressure.begin();
  step.begin();
  mag.begin();
  airq.begin();
  airbsec.begin();
  gas.begin();
  temp.begin();
  delay(4000);
}

long check_timestamp = 1000;
long millis_check = 0;
bool start_timestamp = true;

float gasquality_start;
float temperature_start;

bool gas_inside = true;
bool temp_inside = true; 

int InsideOutside(){

  if(start_timestamp){
    gasquality_start = float(gas.value());
    temperature_start = float(temp.value());

    millis_check = millis();
    start_timestamp = false;
  }

  if(millis() - millis_check > check_timestamp){
    float delta_gasquality = float(gas.value()) - gasquality_start;
    float delta_tempquality = float(temp.value()) - temperature_start;

    Serial.println(String(delta_gasquality));
    Serial.println(String(delta_tempquality));
    //Serial.println(String(gasquality_start));
    //Serial.println(String(float(gas.value())));

    if((abs(delta_gasquality) > (float) THRESHOLD_GAS) && delta_gasquality > 0.0){
      gas_inside = false;
    }

    if((abs(delta_gasquality) > (float) THRESHOLD_GAS) && delta_gasquality < 0.0){
      gas_inside = true;
    }

    if((abs(delta_tempquality) > (float) THRESHOLD_TEMP) && delta_tempquality > 0.0){
      temp_inside = false;
    }

    if((abs(delta_tempquality) > (float) THRESHOLD_TEMP) && delta_tempquality < 0.0){
      temp_inside = true;
    }
   
    start_timestamp = true;
   
    if(gas_inside && temp_inside) return 2;
    if(gas_inside && !temp_inside) return 1;
    if(!gas_inside && temp_inside) return -1;
    if(!gas_inside && !temp_inside) return -2;
  }
  return 0;
}




void loop(){
  BHY2.update();
  // Check sensor values every second  
  int innout = InsideOutside();
  if(innout != 0) Serial.println((int) innout);
}
