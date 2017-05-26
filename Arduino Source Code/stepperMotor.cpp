#include "Arduino.h"
#include <Arduino_FreeRTOS.h>
#include "StepperMotor.h"

#define SLOW_SPEED 32    //32ms delay
#define MEDIUM_SPEED 16  //16ms delay
#define FAST_SPEED 0     //0ms delay

StepperMotor::StepperMotor()
{
  stepCountMax = 0;
  stepCountMid = 0;
  stepCountCurrent = 0;

  calibrationInProgressFlag = false;
  calibrationCompletionFlag = false;
}

StepperMotor::~StepperMotor()
{
  
}

bool StepperMotor::assignMotorConnections(int limitSwitchPin, int motorEnablePin, int motorDrivePin, int motorDirectionPin)
{
  pinMode(limitSwitchPin, INPUT);
  _limitSwitchPin = limitSwitchPin;
  
  pinMode(motorEnablePin, OUTPUT);
  _motorEnablePin = motorEnablePin;

  pinMode(motorDrivePin, OUTPUT);
  _motorDrivePin = motorDrivePin;
  digitalWrite(_motorDrivePin, LOW); //Preset drive step to stepper motor board to LOW

  pinMode(motorDirectionPin, OUTPUT);
  _motorDirectionPin = motorDirectionPin;

//  pinMode(motorStepPin, OUTPUT);
//  _motorStepPin = motorStepPin;

}

bool StepperMotor::presetMotor(MotorStatus mStatus, MotorSpeed mSpeed, MotorStepSize mStep, MotorDirection mDirection)
{
  /* Active-Low to enable Stepper Motor Driver board*/
  switch(mStatus){
    case ON: digitalWrite(_motorEnablePin, LOW); break;   //Enable
    case OFF: digitalWrite(_motorEnablePin, HIGH); break; //Disable
    default: digitalWrite(_motorEnablePin, HIGH); break;  //Disable
  }

  switch(mSpeed){
    case SLOW: _motorSpeed = SLOW_SPEED; break;
    case MEDIUM: _motorSpeed = MEDIUM_SPEED; break;
    case FAST: _motorSpeed = FAST_SPEED; break;
    default: _motorSpeed = SLOW_SPEED; break;
  }

  /* Active-Low: Making it LOW will trigger full step-size*/
//  switch(mStep){
//    case FULL: digitalWrite(_motorStepPin, LOW); break;   //Full Step Size
//    case HALF: digitalWrite(_motorStepPin, HIGH); break;  //Half Step Size
//    default: digitalWrite(_motorStepPin, LOW); break;     //Full Step Size
//  }

  switch(mDirection){
    case FORWARD: digitalWrite(_motorDirectionPin, HIGH); break; //Just naming it forward. It can be configured vice-versa.
    case REVERSE: digitalWrite(_motorDirectionPin, LOW); break;
    default: digitalWrite(_motorDirectionPin, HIGH); break;
  }

  return true;
}

int StepperMotor::setMotorSpeed(MotorSpeed mSpeed)
{
  switch(mSpeed){
    case SLOW: _motorSpeed = SLOW_SPEED; return 1;
    case MEDIUM: _motorSpeed = MEDIUM_SPEED; return 2;
    case FAST: _motorSpeed = FAST_SPEED; return 3;
    default: _motorSpeed = MEDIUM_SPEED; return 0;
  }
}

//int StepperMotor::setMotorStepSize(MotorStepSize mStep)
//{
//  /* Active-Low: Making it LOW will trigger full step-size*/
//  switch(mStep){
//    case FULL: digitalWrite(_motorStepPin, LOW); return 1;  //Full Step Size
//    case HALF: digitalWrite(_motorStepPin, HIGH); return 2; //Half Step Size
//    default: digitalWrite(_motorStepPin, HIGH); return 0;   //Full Step Size
//  }
//}

int StepperMotor::setMotorDirection(MotorDirection mDirection)
{
    switch(mDirection){
    case FORWARD: digitalWrite(_motorDirectionPin, HIGH); return 1;//Just naming it forward. It can be configured vice-versa.
    case REVERSE: digitalWrite(_motorDirectionPin, LOW); return 2;
    default: digitalWrite(_motorDirectionPin, HIGH); return 0;
  }
}

void StepperMotor::driveMotor(MotorSpeed mSpeed, MotorStepSize mStep, MotorDirection mDirection)
{
  this->setMotorSpeed(mSpeed);
//  this->setMotorStepSize(mStep);
  this->setMotorDirection(mDirection);
  
  digitalWrite(_motorEnablePin, LOW);  //Enable motor if not already enabled
  digitalWrite(_motorDrivePin, HIGH);  //Sends HIGH pulse to Stepper Motor Drive to turn 1 step
  digitalWrite(_motorDrivePin, LOW);
  
  vTaskDelay( _motorSpeed / portTICK_PERIOD_MS ); //Delay is to control motor speed
}

void StepperMotor::driveMotorClosedLoop(MotorSpeed mSpeed, MotorStepSize mStep, MotorDirection mDirection)
{
  if(this->calibrationCompletionFlag == true)
  {
    if((mDirection == FORWARD) && ((this->stepCountCurrent < this->stepCountMax) || (checkLimitSwitch() == false)))
    {
      driveMotor(mSpeed, mStep, FORWARD);
      this->stepCountCurrent++;
    }
    else if((mDirection == REVERSE) && (this->stepCountCurrent > 0))
      {
        driveMotor(mSpeed, mStep, REVERSE);
        this->stepCountCurrent--;
      }   
  }
  else if(this->calibrationCompletionFlag == false)
    {
      driveMotor(mSpeed, mStep, mDirection);
    }
}


bool StepperMotor::checkLimitSwitch()
{
  if(digitalRead(_limitSwitchPin) == HIGH)
    return true;
  else
    return false;
}

bool StepperMotor::stopMotor()
{
  digitalWrite(_motorEnablePin, HIGH);  //Disable Motor
  return true;
}

