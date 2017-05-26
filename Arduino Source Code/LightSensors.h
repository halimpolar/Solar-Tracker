/*
  LightSensors.h - Use to read photo resistor values and measure solar panel voltage output.
  Created by Yang Thao, 4/8/2017.
*/

#ifndef LightSensors_h
#define LightSensors_h

class LightSensors{
  public:
  LightSensors();
  virtual ~LightSensors();
  bool assignSensorPins(int prTopRight, int prTopLeft, int prBottomRight, int prBottomLeft, int solarPanel);
  int getHorizontalError();
  int getVeriticleError();
  int getSolarPanelReading();
  
  private:

  int _prTopRight;
  int _prTopLeft;
  int _prBottomRight;
  int _prBottomLeft;
  int _solarPanel;
  
  bool assignAnalogInputPin(int pinNum);
  int readAnalogPin(int pinNum);
};

#endif
