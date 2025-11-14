/***************************************************************
 *  Credits:
 *  Oussama Errouji
 *  Developed and adapted this code during the NRC Competition,
 *  2025 edition.
 *
 *  File Purpose:
 *  Main program for the ROBOTX-8DOF quadruped robot. Handles full system integration:
 *  initialization, hardware control, state management, command processing, and gait cycles.
 *  This file is the entry point and required for overall operation.
 *
 *  Please cite this contribution if reused or reworked.
 ***************************************************************/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "includes.h"

// STATE VARIABLES
boolean MOVING = false;
boolean STOPPING = false;
int STEPS_IN_WALK = 4;
int CURRENT_WALK = LATERAL_WALK;
int CURRENT_POSE;
int NEXT_POSE;
int CURRENT_STEP = 0;
int GRANULARITY = 20; //10;
int CURRENT_TICK = 0;
int STEP_COUNT = 0;
int WALK_CYCLE_COUNT = 0;
int TICK_DELAY = 50;
struct point CURRENT_TRAJECTORY[4][10];
boolean CHAT = false;

void setup() 
{
  Serial.begin(9600);
  Serial.println("ROBOTX-8DOF V0.1");
  
  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
  
  for(int i=0;i < 4;i++)
  {
	legs[i] = make_leg(i);
  }
 
  //STAND 
  Stand();
  
  //test_reverse_trajectory();
}

void loop() 
{
    communicating();
    moving();
}

void communicating()
{
   //if we have a complete message flush the package
   //so its ready for a new command
   if(message_complete) 
   {
      message_complete = false;
      message = "";   
   } 
}

void moving()
{
  //check if we are moving....
  if(!MOVING) return;
  walking();
}

void serialEvent() 
{
  int i=0;
  if (Serial.available()) 
  {
    delay(50);
    while(Serial.available()) {
      int inChar = Serial.read();
      message+= (char)inChar;
    }
    message_complete = true;
    command();
  }
}

void command()
{

  Serial.print("CMD:");
  Serial.println(message);
  
  
  if(message.startsWith("MV")) MVAction();
  if(message.startsWith("SP")) SPAction();
  if(message.startsWith("RX")) RXAction();
  if(message.startsWith("RY")) RYAction();
  if(message.startsWith("OX")) OXAction();
  if(message.startsWith("OY")) OYAction();

}

void OXAction()
{
   int leg_idx = message.substring(2,3).toInt();
   int val = message.substring(3).toInt();
   legs[leg_idx -1].flight.origin.x = val;
}

void OYAction()
{
    int leg_idx = message.substring(2,3).toInt();
   int val = message.substring(3).toInt();
   legs[leg_idx -1].flight.origin.y = val;
}

void RYAction()
{
    int leg_idx = message.substring(2,3).toInt();
    int val = message.substring(3).toInt();
    legs[leg_idx -1].flight.radius.y = val;
 
}

void RXAction()
{
    int leg_idx = message.substring(2,3).toInt();
    int val = message.substring(3).toInt();

    legs[leg_idx -1].flight.radius.x = val; 
}

void SPAction()
{
   int val = message.substring(2).toInt();
   TICK_DELAY = val;
}

void MVAction()
{
  int val = message.substring(2).toInt();
  if(val==1) Play();
  if(val==0) Stop();
  if(val==2) StepForward();
  if(val==3) Stand(); //StepBackward();
}

void Stop()
{
    STOPPING = true;
}
void Play()
{
    MOVING = true;
        for (int i = 0; i < 4; i++) {
        legs[i].flight.radius.x = 45; // increase forward/backward step size
        legs[i].flight.radius.y = 60; // increase vertical lift
    }
}

void Stand()
{
  MOVING = false;
  for(int i=0;i < 4;i++)
  {            
    set_leg(i,STAND[i]);
    //if(LegNames[legs[i].index] == "FRONT RIGHT")
    //{
      Serial.println("");
      Serial.print("Leg ");
      Serial.print(LegNames[legs[i].index]);
      Serial.print(" hip:");
      Serial.print(legs[i].hipknee.hip);
      Serial.print(" knee:");
      Serial.println(legs[i].hipknee.knee);
    //}
  }
}

void StepForward()
{
   for(int i=0;i < 4;i++)
  {
	legs[i].forward = true;
  } 
  MOVING = true;
  STOPPING = true;
}
void StepBackward()
{
  for(int i=0;i < 4;i++)
  {
	legs[i].forward = false;
  } 
  MOVING = true;
  STOPPING = true;
}


void walking()
{
    //this is the first tick on this step
    //find the current pose 
    if(CURRENT_TICK == 0)
    { 
        if(CHAT) Serial.println("Current tick:0 - find trajectory");
        for(int i=0;i < 4;i++)
        {            
            leg_trajectory_for_step(&legs[i],CURRENT_STEP);
        }
    }
  
    //position the legs based on the current tick and place on the current leg trajectory plan
    if(CHAT) Serial.print("Position leg to tick:");
    if(CHAT) Serial.println(CURRENT_TICK);
    
    for(int i=0;i < 4;i++)
    {
        leg_position_for_tick(&legs[i]);
    }
    
    //increase the tick
    CURRENT_TICK++;

    if(CURRENT_TICK >= GRANULARITY) //new step
    {
        STEP_COUNT++;
        CURRENT_STEP++;
        CURRENT_TICK = 0;
        
        if(CHAT) Serial.println("************* step completed *****************");
    }
    if(CURRENT_STEP >= STEPS_IN_WALK) //new cycle
    {
        WALK_CYCLE_COUNT++;
        CURRENT_STEP = 0;
        if(CHAT) Serial.println("*************** Cycle completed ****************");
               
        if(STOPPING)
        {         
          MOVING = false;
          STOPPING = false;
        }
    }
    delay(TICK_DELAY);
 
}
