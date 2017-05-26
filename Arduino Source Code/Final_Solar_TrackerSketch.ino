#include <Arduino_FreeRTOS.h>

/* Requires too much memory resources. Will not work on this program */
//#include <Bridge.h>
//#include <BridgeServer.h>
//#include <BridgeClient.h> 
//BridgeServer server;
//BridgeClient client;

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#include "ProjectData.h"
#include "StepperMotor.h"
#include "Manager.h"

#define HORIZONTAL_ANGLE 225
#define VERITICAL_ANGLE 90
#define SOLAR_FACTOR 0.005

#define BUTTON_GO_LEFT 0
#define BUTTON_GO_RIGHT 1
#define BUTTON_GO_UP 12
#define BUTTON_GO_DOWN 13

/* Setup for LCD Display */
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

/* Define tasks that will run parallelly together */
void TaskSolarAndLCD( void *pvParameters );
void TaskDriveMotorHorizontal( void *pvParameters );
void TaskDriveMotorVerticle( void *pvParameters );
void TaskInitializeHorizontalMotorAxis( void *pvParameters );
void TaskInitializeVeriticleMotorAxis( void *pvParameters );
void TaskRunSystem( void *pvParameters );

/* Creates instance of a Manager class to handle everything */
Manager *my_Manager = new Manager();

/* The setup function runs once when you press reset or power the board */
void setup() {

  /* Initializes Buttons */
  pinMode(BUTTON_GO_LEFT, INPUT_PULLUP);
  pinMode(BUTTON_GO_RIGHT, INPUT_PULLUP);
  pinMode(BUTTON_GO_UP, INPUT_PULLUP);
  pinMode(BUTTON_GO_DOWN, INPUT_PULLUP);

  /* Initialize the lcd for 20 chars 4 lines, turn on backlight */
  lcd.begin(20,4);
  
  /* Initialize serial communication at 9600 bits per second */
  Serial.begin(9600);

  /* Startup bridge listening to website call */
//  Bridge.begin();
//  server.listenOnLocalhost();
//  server.begin();
 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }
 
  /* Now set up at least two or more tasks to run independently */
  xTaskCreate(
    TaskRunSystem
    ,  (const portCHAR *)"Run Solar Tracking System"   // A name just for humans
    ,  128 // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
    
  xTaskCreate(
    TaskInitializeHorizontalMotorAxis
    ,  (const portCHAR *)"Horizontal Axis Initialization"   // A name just for humans
    ,  96  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskInitializeVeriticleMotorAxis
    ,  (const portCHAR *)"Veritcle Axis Initialization"   // A name just for humans
    ,  96  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
    
  xTaskCreate(
    TaskDriveMotorHorizontal
    ,  (const portCHAR *)"HorizontalStepperMotor"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskDriveMotorVerticle
    ,  (const portCHAR *)"VerticleStepperMotor"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
 
  xTaskCreate(
    TaskSolarAndLCD
    ,  (const portCHAR *) "Solar Panel voltage"
    ,  192  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}
 
void loop()
{
  // Empty. Things are done in Tasks.
}
 
/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
 
void TaskSolarAndLCD(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  double solarReading, solarVoltage, horizontalFactor, horizontalAngle, verticalFactor, verticalAngle;

/* Preset the LCD screen once */
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Solar Tracker System");
  lcd.setCursor(0,1);
  lcd.print("Solar Voltage:");
  lcd.setCursor(0,2);
  lcd.print("Horizontl Pos:");
  lcd.setCursor(0,3);
  lcd.print("Vertical  Pos:");
 
 int solarValue = 0;
 
  for (;;)
  {
     /*  Updates Solar Voltage on LCD */
    solarReading = (double)my_Manager->my_LightSensors->getSolarPanelReading();
    solarVoltage = solarReading * SOLAR_FACTOR;
    lcd.setCursor(15,1);
    lcd.print("     ");
    lcd.setCursor(15,1);
    lcd.print(solarVoltage);

    /* Updates Horizontal Axis in degrees on LCD*/
    if((my_Manager->my_StepperMotor_H->calibrationCompletionFlag == true))
    {
      horizontalFactor =  HORIZONTAL_ANGLE / (double)my_Manager->my_StepperMotor_H->stepCountMax;
      horizontalAngle = (double)my_Manager->my_StepperMotor_H->stepCountCurrent * horizontalFactor;
      lcd.setCursor(14,2);
      lcd.print("      ");
      lcd.setCursor(14,2);
      lcd.print(horizontalAngle);
    }
    else if((my_Manager->my_StepperMotor_H->calibrationCompletionFlag != true))
    {
      lcd.setCursor(15,2);
      lcd.print(" N/A ");
    }

    /* Updates Vertical Axis in degrees on LCD */
    if((my_Manager->my_StepperMotor_V->calibrationCompletionFlag == true))
    {
      verticalFactor = VERITICAL_ANGLE / (double)my_Manager->my_StepperMotor_V->stepCountMax;
      verticalAngle = (double)my_Manager->my_StepperMotor_V->stepCountCurrent * verticalFactor;
      lcd.setCursor(14,3);
      lcd.print("     ");
      lcd.setCursor(14,3);
      lcd.print(verticalAngle);
    }
    else if((my_Manager->my_StepperMotor_V->calibrationCompletionFlag != true))
    {
      lcd.setCursor(15,3);
      lcd.print(" N/A ");
    }

    /* Reads voltage from solar panel */
    /* read the input on analog pin 4: */
    solarValue = my_Manager->my_LightSensors->getSolarPanelReading();
    // print out the value you read:
    Serial.println(solarValue);

    /* Check communication with website. Send result back if request has been made */
    /* Due to limited memory resources, this feature will not run */
//    client = server.accept();
//    if (client) {
//    client.print(solarValue*0.005);
//    }
//   client.stop();
   
    vTaskDelay(100 / portTICK_PERIOD_MS);  // one tick delay (15ms) in between reads for stability
  }
}

/*This task will manually drive Horizontal axis motor when switches are pressed */
void TaskDriveMotorHorizontal(void *pvParameters)
{
  (void) pvParameters;

  int statusRightButton, statusLeftButton;
  
  for(;;)
  {
    if(my_Manager->my_StepperMotor_H->calibrationInProgressFlag != true)
    {
      statusRightButton = digitalRead(BUTTON_GO_RIGHT);
      statusLeftButton = digitalRead(BUTTON_GO_LEFT);

      if( (statusRightButton == HIGH) && (statusLeftButton == LOW) )
        {
        /* (speed, step, direction) */
        my_Manager->my_StepperMotor_H->driveMotorClosedLoop(FAST, FULL, FORWARD);
        }
      else if( (statusRightButton == LOW) && (statusLeftButton == HIGH) )
        {
        /* (speed, step, direction) */
        my_Manager->my_StepperMotor_H->driveMotorClosedLoop(FAST, FULL, REVERSE);
        }
      else{ /* Do Nothing */}
    }
    vTaskDelay( 16 / portTICK_PERIOD_MS );  // tick delay (per ms) in between reads for stability (16ms is the lowest to be stable)
  }
}

/* This task will manually drive Vertical axis motor when switches are pressed */
void TaskDriveMotorVerticle(void *pvParameters)
{
  (void) pvParameters;

  int statusUpButton, statusDownButton;
  
  for(;;)
  {
    statusUpButton = digitalRead(BUTTON_GO_UP);
    statusDownButton = digitalRead(BUTTON_GO_DOWN);
    
    if(my_Manager->my_StepperMotor_V->calibrationInProgressFlag != true)
    {
      if( (statusDownButton == HIGH) && (statusUpButton == LOW) )
        {
          /* (speed, step, direction) */
          my_Manager->my_StepperMotor_V->driveMotorClosedLoop(FAST, FULL, FORWARD);
        }
    else if( (statusDownButton == LOW) && (statusUpButton == HIGH) )
      {
         /* (speed, step, direction) */
          my_Manager->my_StepperMotor_V->driveMotorClosedLoop(FAST, FULL, REVERSE);
      }
    else{ /* Do Nothing */}
    }
    vTaskDelay( 16 / portTICK_PERIOD_MS );  // tick delay (per ms) in between reads for stability (16ms is the lowest to be stable)
  }
}

/* Initializes Horizontal motor axis and calculates its position */
void TaskInitializeHorizontalMotorAxis(void *pvParameters)
{
  (void) pvParameters;
  int buttonStatus;

  pinMode(A5, INPUT_PULLUP);

  for(;;)
  {
    buttonStatus = analogRead(A5);
    
    if(buttonStatus <= 20)
    {
      my_Manager->initAxis_Motor(my_Manager->my_StepperMotor_H);
    }

    vTaskDelay( 16 / portTICK_PERIOD_MS );  // tick delay (per ms) in between reads for stability (16ms is the lowest to be stable)
  }
}

/* Initializes Vertical motor axis and calcualtes its position */
void TaskInitializeVeriticleMotorAxis(void *pvParameters)
{
  (void) pvParameters;
  int buttonStatus;

  pinMode(A5, INPUT_PULLUP);

  for(;;)
  {
    buttonStatus = analogRead(A5);
    
    if(buttonStatus <= 20)
    {
      my_Manager->initAxis_Motor(my_Manager->my_StepperMotor_V);
    }

    vTaskDelay( 16 / portTICK_PERIOD_MS );  // tick delay (per ms) in between reads for stability (16ms is the lowest to be stable)
  }
}

/* Once both motr axis calibraiton are done, this task starts the auto tracking feature */
void TaskRunSystem(void *pvParameters)
{
  (void) pvParameters;
  
  for(;;)
  {
    if((my_Manager->my_StepperMotor_H->calibrationInProgressFlag != true) && (my_Manager->my_StepperMotor_V->calibrationInProgressFlag != true))
    {
      my_Manager->runSystem();
    }
    vTaskDelay( 16 / portTICK_PERIOD_MS );  // tick delay (per ms) in between reads for stability (16ms is the lowest to be stable)
  }
}

