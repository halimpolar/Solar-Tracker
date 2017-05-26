#include "Arduino.h"
#include <Arduino_FreeRTOS.h>
#include "Manager.h"

#define STARTUP_STEPS 500

#define UPPER_LIMIT 10
#define LOWER_LIMIT -10

Manager::Manager(){
  my_StepperMotor_H = new StepperMotor();
  my_StepperMotor_V = new StepperMotor();
  my_LightSensors = new LightSensors();

  /*Assigns pins for each motor axis */
  /*Parameters (int limitSwitchPin, int motorEnablePin, int motorDrivePin, int motorDirectionPin)*/
  my_StepperMotor_H->assignMotorConnections(4,5,6,7);
  my_StepperMotor_V->assignMotorConnections(8,9,10,11);


  /*Assings pins for each Photo Resistors and Solar Panel output*/
  /*Parameters (int prTopRight, int prTopLeft, int prBottomRight, int prBottomLeft, int solarPanel)*/
  my_LightSensors->assignSensorPins(0,1,2,3,4);

  /* Preset Motor Parameters */
  /* MotorSpeed mSpeed, MotorStepSize mStep, MotorDirection mDirection */
  my_StepperMotor_H->presetMotor(ON, FAST, FULL, FORWARD);
  my_StepperMotor_V->presetMotor(ON, FAST, FULL, FORWARD);
  
}

Manager::~Manager()
{
  
}

bool Manager::initAxis_Motor(StepperMotor *Motor)
{
  Motor->calibrationCompletionFlag = false;
  Motor->calibrationInProgressFlag = true;
  
  int i;
  int totalSteps = 0;

  for(i=0;i<STARTUP_STEPS;i++)
  {
    Motor->driveMotor(FAST, FULL, REVERSE);
    vTaskDelay( 16 / portTICK_PERIOD_MS );  // tick delay (per ms) in between reads for stability (16ms is the lowest to be stable)
  }

  while(Motor->checkLimitSwitch() == false)
  {
    Motor->driveMotor(FAST, FULL, FORWARD);
    totalSteps++;
    vTaskDelay( 16 / portTICK_PERIOD_MS );  // tick delay (per ms) in between reads for stability (16ms is the lowest to be stable)
  }

  Motor->stepCountMax = totalSteps;
  Motor->stepCountMid = (totalSteps/2);
  Motor->stepCountCurrent = totalSteps;

  for(i=0; i < Motor->stepCountMid; i++)
  {
    Motor->driveMotor(FAST, FULL, REVERSE);
    Motor->stepCountCurrent--;
    vTaskDelay( 16 / portTICK_PERIOD_MS );  // tick delay (per ms) in between reads for stability (16ms is the lowest to be stable)
  }
  
  //Serial.println(Motor->stepCountMax);
  //Serial.println(Motor->stepCountCurrent);

  Motor->calibrationCompletionFlag = true;
  Motor->calibrationInProgressFlag = false;
  
  return true;
}

bool Manager::runSystem()
{
  int h_Error;
  int v_Error;
  
  if((my_StepperMotor_H->calibrationCompletionFlag == true) && (my_StepperMotor_V->calibrationCompletionFlag == true))
  {
    h_Error = my_LightSensors->getHorizontalError();
    v_Error = my_LightSensors->getVeriticleError();

    if(h_Error > UPPER_LIMIT)
      my_StepperMotor_H->driveMotorClosedLoop(FAST, FULL, FORWARD);  //Moves Left
    else if( h_Error < LOWER_LIMIT)
      my_StepperMotor_H->driveMotorClosedLoop(FAST, FULL, REVERSE);  //Moves Right

    if(v_Error > UPPER_LIMIT)
      my_StepperMotor_V->driveMotorClosedLoop(FAST, FULL, REVERSE);  //Moves Up
    else if(v_Error < LOWER_LIMIT)
      my_StepperMotor_V->driveMotorClosedLoop(FAST, FULL, FORWARD);  //Moves Down
  }
}

