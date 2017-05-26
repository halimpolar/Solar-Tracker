/*
  Manager.h - Use to manager whole system.
  Created by Yang Thao, 3/27/2017.
*/
#ifndef Manger_h
#define Manger_h

#include "ProjectData.h"
#include "StepperMotor.h"
#include "LightSensors.h"

class Manager{
  public:
  Manager();
  virtual ~Manager();
  StepperMotor *my_StepperMotor_H;
  StepperMotor *my_StepperMotor_V;
  LightSensors *my_LightSensors;
  
  bool initAxis_Motor(StepperMotor *Motor);
  bool runSystem();

  private:
  
  };

#endif
