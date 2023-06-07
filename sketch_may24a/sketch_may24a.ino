//Tim Herrigt, Ulrich Seemann

#include "Arduino_BHY2.h"
#include "Nicla_System.h"

#define THRESHOLD_GAS 600
#define THRESHOLD_TEMP 0.3
#define THRESHOLD_PRESSURE 0.05

Sensor pressure(SENSOR_ID_BARO);
Sensor step(SENSOR_ID_STC);
Sensor airq(SENSOR_ID_HUM);
Sensor airbsec(SENSOR_ID_BSEC);
Sensor gas(SENSOR_ID_GAS);
Sensor temp(SENSOR_ID_TEMP);

void setup() {
  Serial.begin(115200);
  BHY2.begin();

  nicla::begin();
  nicla::leds.begin();

  pressure.begin();
  step.begin();
  airq.begin();
  airbsec.begin();
  gas.begin();
  temp.begin();

  delay(8000);

  Serial.println("Start: ");
}

long check_timestamp_es = 2000;
long millis_check_es = 0;
bool start_timestamp_es = true;

float pressure_start_es = 0;
int step_start_es = 0;

float delta_pressure_old_es = 0; 
int delta_steps_old_es = 0;

bool pressure_continuity_check_es = false; 
bool steps_continuity_check_es = false;

bool step_check_es = false;
bool elev_check_es = false;

int ElevatorStepps(){

  if(start_timestamp_es){
    pressure_start_es = float(pressure.value());
    step_start_es = int(step.value());

    millis_check_es = millis();
    start_timestamp_es = false;
  }

  if((millis() - millis_check_es) > check_timestamp_es){
    float delta_pressure = float(pressure.value()) - pressure_start_es;
    int delta_steps = int(step.value()) - step_start_es;

    if((abs(delta_pressure) <= (abs(delta_pressure_old_es) + THRESHOLD_PRESSURE)) && (abs(delta_pressure) >= (abs(delta_pressure_old_es) - THRESHOLD_PRESSURE) && (abs(delta_pressure) > (THRESHOLD_PRESSURE)))){
      pressure_continuity_check_es = true; 
    }
    else{
      pressure_continuity_check_es = false;
    }
    
    if((int)step.value() > step_start_es){
      steps_continuity_check_es = true; 
    }
    else{
      steps_continuity_check_es = false;
    }

    if(pressure_continuity_check_es && steps_continuity_check_es){
      step_check_es = true;
      elev_check_es = false;
    }

    if(pressure_continuity_check_es && !steps_continuity_check_es){
      step_check_es = false;
      elev_check_es = true;
    }

    if(!pressure_continuity_check_es){
      step_check_es = false;
      elev_check_es = false;
    }

    start_timestamp_es = true;
    delta_pressure_old_es = delta_pressure;
    delta_steps_old_es = delta_steps;

    if(step_check_es && !elev_check_es) return 1; // Treppe
    if(!step_check_es && elev_check_es) return -1; // Fahrstuhl
    if(!step_check_es && !elev_check_es) return 0;
  }
  return 0;
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

  if(innout == 2 || innout == 1) nicla::leds.setColor(green);  //Drinnen
  if(innout == -2 || innout == -1) nicla::leds.setColor(blue); //Drau0en 

  if(elevstep == 1) nicla::leds.setColor(red); //Treppe
  if(elevstep == -1) nicla::leds.setColor(yellow); //Fahrstuhl

  }
