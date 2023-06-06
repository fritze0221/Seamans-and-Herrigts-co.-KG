#include "Arduino_BHY2.h"

#define THRESHOLD_GAS 500
#define THRESHOLD_TEMP 0.1
#define THRESHOLD_PRESSURE 0.02
#define THRESHOLD_STEP 1

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

long check_timestamp_es = 2000;
long millis_check_es = 0;
bool start_timestamp_es = true;

float pressure_start_es;

int step_es = 0;
float delta_pressure_old_es = 0; 
int delta_steps_old_es = 0;

bool pressure_continuity_check_es = false; 
bool steps_continuity_check_es = false;

bool step_es = false;
bool elev_es = false;

int ElevatorStepps(){

  if(check_timestamp_es){
    pressure_start_es = float(pressure.value());
    step_es = int(step.value());

    millis_check_es = millis();
    check_timestamp_es = false;
  }

  if(millis() - millis_check_es > start_timestamp_es){
    float delta_pressure = float(pressure.value()) - pressure_start_es;
    int delta_steps = int(step.value()) - step_es;

    if((delta_pressure <= (delta_pressure_old_es + THRESHOLD_PRESSURE)) && (delta_pressure >= (delta_pressure_old_es - THRESHOLD_PRESSURE) && (abs(delta_pressure) > (2 * THRESHOLD_PRESSURE)))){
      pressure_continuity_check_es = true; 
    }
    else{
      pressure_continuity_check_es = false;
    }
    
    if((delta_steps <= (delta_steps_old_es + THRESHOLD_PRESSURE)) && (delta_steps >= (delta_steps_old_es - THRESHOLD_PRESSURE) && (abs(delta_steps) > (THRESHOLD_STEP)))){
      steps_continuity_check_es = true; 
    }
    else{
      steps_continuity_check_es = false;
    }

    if(pressure_continuity_check_es && steps_continuity_check_es){
      step_es = true;
      elev_es = false;
    }

    if(pressure_continuity_check_es && !steps_continuity_check_es){
      step_es = false;
      elev_es = true;
    }

    if(!pressure_continuity_check_es){
      step_es = false;
      elev_es = false;
    }

    if(step_es && !elev_es) return 1; // Treppe
    if(!step_es && elev_es) return -1; // Fahrstuhl
    if(!step_es && !elev_es) return 0;

    check_timestamp_es = true;
    delta_pressure_old_es = delta_pressure;
    delta_steps_old_es = delta_steps;
  }
}

long check_timestamp_io = 1000;
long millis_check_io = 0;
bool start_timestamp_io = true;

float gasquality_start_io = 0;
float temperature_start_io = 0;

bool gas_inside_io = true;
bool temp_inside_io = true; 

int InsideOutside(){

  if(start_timestamp_io){
    gasquality_start_io = float(gas.value());
    temperature_start_io = float(temp.value());

    millis_check_io = millis();
    start_timestamp_io = false;
  }

  if(millis() - millis_check_io > check_timestamp_io){
    float delta_gasquality = float(gas.value()) - gasquality_start_io;
    float delta_tempquality = float(temp.value()) - temperature_start_io;

    Serial.println(String(delta_gasquality));
    Serial.println(String(delta_tempquality));
    //Serial.println(String(gasquality_start));
    //Serial.println(String(float(gas.value())));

    if((abs(delta_gasquality) > (float) THRESHOLD_GAS) && delta_gasquality > 0.0){
      gas_inside_io = false;
    }

    if((abs(delta_gasquality) > (float) THRESHOLD_GAS) && delta_gasquality < 0.0){
      gas_inside_io = true;
    }

    if((abs(delta_tempquality) > (float) THRESHOLD_TEMP) && delta_tempquality > 0.0){
      temp_inside_io = false;
    }

    if((abs(delta_tempquality) > (float) THRESHOLD_TEMP) && delta_tempquality < 0.0){
      temp_inside_io = true;
    }
   
    start_timestamp_io = true;
   
    if(gas_inside_io && temp_inside_io) return 2; //Drinnen
    if(gas_inside_io && !temp_inside_io) return 1;
    if(!gas_inside_io && temp_inside_io) return -1;
    if(!gas_inside_io && !temp_inside_io) return -2; //Draußen
  }
  return 0;
}




void loop(){
  BHY2.update();
  // Check sensor values every second  
  int innout = InsideOutside();
  int elevstep = ElevatorStepps();

  if(innout != 0){
    Serial.println("Inside Outside: ")
    Serial.println((int) innout);
  }
  if(elevstep != 0){
    Serial.println("Elevator Stepps: ")
    Serial.println((int) elevstep);
  }

  }
