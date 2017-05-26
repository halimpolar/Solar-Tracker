#include "Arduino.h"
#include "LightSensors.h"

LightSensors::LightSensors()
{

}

LightSensors::~LightSensors()
{
  
}

bool LightSensors::assignAnalogInputPin(int pinNum)
{
  switch(pinNum){
    case 0: pinMode(A0, INPUT); return true;
    case 1: pinMode(A1, INPUT); return true;
    case 2: pinMode(A2, INPUT); return true;
    case 3: pinMode(A3, INPUT); return true;
    case 4: pinMode(A4, INPUT); return true;
    case 5: pinMode(A5, INPUT); return true;
    default: return false;
    }
}

int LightSensors::readAnalogPin(int pinNum)
{
  switch(pinNum){
    case 0: return analogRead(A0);
    case 1: return analogRead(A1);
    case 2: return analogRead(A2);
    case 3: return analogRead(A3);
    case 4: return analogRead(A4);
    case 5: return analogRead(A5);
    default: return 0;
    }
}

bool LightSensors::assignSensorPins(int prTopRight, int prTopLeft, int prBottomRight, int prBottomLeft, int solarPanel)
{
  assignAnalogInputPin(prTopRight);
  _prTopRight = prTopRight;

  assignAnalogInputPin(prTopLeft);
  _prTopLeft = prTopLeft;

  assignAnalogInputPin(prBottomRight);
  _prBottomRight = prBottomRight;

  assignAnalogInputPin(prBottomLeft);
  _prBottomLeft = prBottomLeft;

  assignAnalogInputPin(solarPanel);
  _solarPanel = solarPanel;
  
  return true;
}

int LightSensors::getHorizontalError()
{
  int avgLeft,avgRight, error;

  avgLeft = (readAnalogPin(_prTopLeft) + readAnalogPin(_prBottomLeft)) / 2;
  avgRight = (readAnalogPin(_prTopRight) + readAnalogPin(_prBottomRight)) / 2;
  error = avgLeft - avgRight;

  return error; //If positive, needs to move left. If negative, needs to move right.
}

int LightSensors::getVeriticleError()
{
  int avgTop, avgBottom, error;

  avgTop = (readAnalogPin(_prTopLeft) + readAnalogPin(_prTopRight)) / 2;
  avgBottom = (readAnalogPin(_prBottomLeft) + readAnalogPin(_prBottomRight)) / 2;
  error = avgTop - avgBottom;

  return error; //If positive, needs to move up. If negative, needs to move down.
}

int LightSensors::getSolarPanelReading()
{
  return readAnalogPin(_solarPanel);
}



