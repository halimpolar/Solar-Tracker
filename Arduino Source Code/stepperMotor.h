/*
  stepperMotor.h - Use to drive project stepper motors.
  Created by Yang Thao, 3/27/2017.
  ..\Arduino\libraries\stepperMotor\stepperMotor.h
*/
#ifndef StepperMotor_h
#define StepperMotor_h

#include "ProjectData.h"

class StepperMotor{
  public:
  
  StepperMotor();
  virtual ~StepperMotor();
  bool assignMotorConnections(int limitSwitchPin, int motorEnablePin, int motorDrivePin, int motorDirectionPin);
  bool presetMotor(MotorStatus mStatus, MotorSpeed mSpeed, MotorStepSize mStep, MotorDirection mDirection);
  int setMotorSpeed(MotorSpeed mSpeed);
  int setMotorStepSize(MotorStepSize mStep);
  int setMotorDirection(MotorDirection mDirection);
  void driveMotor(MotorSpeed mSpeed, MotorStepSize mStep, MotorDirection mDirection);
  void driveMotorClosedLoop(MotorSpeed mSpeed, MotorStepSize mStep, MotorDirection mDirection);
  bool checkLimitSwitch();
  bool stopMotor();

  int stepCountMax;
  int stepCountMid;
  int stepCountCurrent;

  bool calibrationInProgressFlag;
  bool calibrationCompletionFlag;
  
  private:
  
  int _motorSpeed;
  int _motorEnablePin;
  int _motorStepPin;
  int _motorDirectionPin;
  int _motorDrivePin;
  int _limitSwitchPin;

  };

#endif
