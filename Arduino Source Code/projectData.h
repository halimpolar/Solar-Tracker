/*
  project_Data.h - ENUM data are declared here
  Created by Yang Thao, 3/27/2017.
  ..\Arduino\libraries\projectData\projectData.h
*/

#ifndef ProjectData_h
#define ProjectData_h

enum MotorStatus{
  ON,
  OFF
};

enum MotorSpeed{
  SLOW,
  MEDIUM,
  FAST
};

enum MotorStepSize
{
  FULL,
  HALF
};

enum MotorDirection
{
  FORWARD,
  REVERSE
};

#endif
