/*

Ringo Robot:  RingoHardware.h  Rev01.01  08/2015

This code was written by Plum Geek LLC primarily 
by Dustin Soodak with some editing and additional 
functions by Kevin King.
Portions from other open source projects where noted.
This code is licensed under:
Creative Commons Attribution-ShareAlike 2.0 Generic (CC BY-SA 2.0)
https://creativecommons.org/licenses/by-sa/2.0/

Visit http://www.plumgeek.com for Ringo information.
Visit http://www.arduino.cc to learn about the Arduino.

*/

#ifndef RINGO_HARDWARE_H
#define RINGO_HARDWARE_H

#include "Arduino.h"
#include "FunStuff.h"
#include "Navigation.h"
#include <Adafruit_NeoPixel.h>


// ***************************************************
// Pin defines
// ***************************************************

#define Accel_Interrupt 2 //used by both Gyro and Accel chips.
#define Accel_Interrupt_Num 0 //pin2 is interrupt 0 on arduino uno board
#define MotorDirection_Right 1
#define MotorDirection_Left 0
#define MotorDrive_Left 6
#define MotorDrive_Right 5
//
#define Chirp 9 //tone(pin, frequency) and noTone(),  or tone(pin, frequency, duration). also look at toneAC library
#define Edge_Lights 8 //turn on IR_FRNT_LEFT_BTM and IR_FRNT_RGHT_BTM
#define _38kHz_Rx 3
#define LightSense_Rear 3 //AD3
#define Source_Select 4 
#define LightSense_Left 2 //AD2 //Source_Select LOW=AMB_FRNT_LEFT, HIGH=EDGE_FRNT_LEFT
#define LightSense_Right 1 //AD1 //Source_Select LOW=AMB_FRNT_RIGHT, HIGH=EDGE_FRNT_RIGHT
#define MotorCapBattVolts 0 //AD0 //Source_Select LOW=motor capacitor, HIGH=battery
//
#define IR_Enable_Front 13
#define IR_Enable_RearLeft 12
#define IR_Enable_RearRight 11
#define IR_Send 10
//
#define Light_Bus_BTN1 7 //for 6 neo pixel RGB

// ***************************************************
// end Pin defines
// ***************************************************

// ***************************************************
// General hardware
// ***************************************************
#define SERIAL_SPEED 57600
extern void HardwareBegin(void);
//Note: at some point might want to incorperate "Switch" functions directly
//into pixel/button functions. Could do this with motor/serial as well but that
//would require modification of the standard Arduino serial library.
extern void SwitchButtonToPixels(void);
extern void SwitchPixelsToButton(void);
extern void SwitchSerialToMotors(void);
extern void SwitchMotorsToSerial(void);
extern char ButtonPressed(void);
//Note: "while(!Serial.available());  delus=Serial.parseInt();" to wait for numerical input from user
// ***************************************************
// end General hardware
// ***************************************************

// ***************************************************
// Simple Timer
// ***************************************************
  //Ultra-simple stop watch functions using the built-in arduino millis() function.
  extern int32_t GetTime(void);
  extern void RestartTimer(void);
  extern void StopTimer(void);
// ***************************************************
// end Simple Timer
// ***************************************************


// ***************************************************
// Pixels
// ***************************************************
#define NUM_PIXELS 6
extern void SetPixelRGB(int Pixel, int Red, int Green, int Blue); // primary function for controlling NeoPixel lights
extern void SetAllPixelsRGB(int Red, int Green, int Blue);//0-255 // sets all pixels to same values. automatically refreshes pixels
extern void RefreshPixels(void);
//Ex: SwitchButtonToPixels();SetPixelRGB(1,100,100,100);RefreshPixels();
//will turn pixel 1 white at a bit less than half brightness 
//0 is tail (rear) top, 1 and 2 are bottom "mood lighting", and 3 is body (front) top
//4 is right headlight/eye, and 5 is left headlight/eye

#define EYE_LEFT 5
#define EYE_RIGHT 4 
#define BODY_TOP 3
#define BODY_BOTTOM 2
#define TAIL_TOP 0
#define TAIL_BOTTOM 1

//
//low level/misc:
//
extern Adafruit_NeoPixel pixels;// = Adafruit_NeoPixel(NUM_PIXELS, Light_Bus_BTN1, NEO_GRB + NEO_KHZ800);
// ***************************************************
// end Pixels
// ***************************************************


// ***************************************************
// Motor
// ***************************************************
#define MOTOR_MAX 255 //motor goes from -255 to 255
extern int LeftMotor;//current motor speed
extern int RightMotor;//current motor speed
extern void MotorsBegin(void);
extern void Motors(int LeftMotorSpeed, int RightMotorSpeed);//-255 to 255: negative values to make it go backwards
// ***************************************************
// end Motor
// ***************************************************


// ***************************************************
// Movement functions
// ***************************************************
//
//To use movement functions, user code must first call "NavigationBegin()", which initializes
//the navigation sensors (though some of them call it automatically). 
//Most of these call "CalibrateNavigationSensors()" in beginning.
//NavigationBegin() calls CalibrateNavigationSensors() for you.
//Navigation will be innacurate if robot is moving when calling NavigationBegin() or
//CalibrateNavigationSensors(). 
//
//Since gravity is a much larger acceleration than it will usually accelerate in
//the XY plane, even slight tipping forwards and backwards will have a huge effect
//if the average tilt while running is significantly different than when the
//nagigation sensors were calibrated.
//
//If 0 entered for EdgeFunction, the move functions will not look for edges.
//If something like Backup with definition void BackUp(void){Motors(-200,-200);}
//is entered, then it will be called when an edge is detected.
//

extern int HeadingWithShortestDistance(int Heading);
//Figures out which of 2 directions (left or right) to turn (in order to get 
//from current direction to Heading.
//Ex: if currently 0 degrees, then HeadingWithShortestDistance(300) will
//return -60 degrees.

extern void MoveXYWithOptions(int X, int Y, int Speed, int MaxExpectedRunTime, int MaxExpectedSkidTime, void (*EdgeFunction)(char), char Wiggle);
//  Goes towards destination {X,Y}.
//    However, this function calculates with both x and y acceleration so is not
//    affected by the Wiggle parameter.
//  ex: ZeroNavigation();MoveXYWithOptions(0,200,150,3000,500,0,50);
//    Since navigation zeroed, it is at coordinates {0,0} and facing 0 degrees (Pi/2 radians) along y-axis.
//    It will wiggle back and forth but still should stop after its y-coordinate reaches
//    approximately 200mm. 
//  In general, this function draws a finish line that is perpendicular to
//    the route from the initial to final {x,y} position. It then continuously
//    resets the "heading" parameter so that it is facing towards the final position
//    and stops when the original finish line is crossed.
//

void MoveWithOptions(int Heading, int Distance, int Speed, int MaxExpectedRunTime, int MaxExpectedSkidTime, void (*EdgeFunction)(char), char Wiggle);
//  Translates Heading & Distance to X,Y and calls MoveXYWithOptions()

extern void RotateSimple(int TurnDegrees, int left, int right, int MaxExpectedTurnTime, int MaxExpectedSkidTime);
//  ex: RotateSimple(GetDegrees()+90, 150, 0, 1000, 250);//approximate rotation
//    Set left motor to 150 and right motor to 0. Stop both motors when the 
//    heading we expect to be at after skidding to a halt is 90 degrees to the
//    right of its initial heading.
//  Note: this uses GetDegreesToStop() which looks at how fast it is currently
//    rotating (GetDegreesPerSecond()) and estimates how much further it will
//    go if motors are turned off right now.
//  
extern char RotateAccurate(int Heading, int MaxExpectedTurnTime);
//  ex: RotateAccurate(45,1000);
//    exact rotation (45 degrees to right if ZeroNavigation() just called) with timeout of 1000ms
//

// ***************************************************
// end Movement functions
// ***************************************************



// ***************************************************
// IR
// ***************************************************


//The following was used as reference code in the development of the IR receive functions:
//http://playground.arduino.cc/Code/InfraredReceivers by Paul Malmsten 
//https://github.com/z3t0/Arduino-IRremote by Ken Shirriff
extern void TxIR(unsigned char *Data, int Length);
//
extern void RxIRStop(void);
extern void RxIRRestart(char BytesToLookFor);
extern char IsIRDone(void);
extern byte GetIRButton(void);
extern char IRNumOfBytes;
//
extern byte GetIRButton(void);
extern const uint8_t IRRemoteButtons[][2];
#define IR_1 1
#define IR_2 2
#define IR_3 3
#define IR_4 4
#define IR_5 5
#define IR_6 6
#define IR_7 7
#define IR_8 8
#define IR_9 9
#define IR_0 10
#define IR_Forward 11
#define IR_Left 12
#define IR_Right 13
#define IR_Backward 14
#define IR_Power 15
#define IR_PlumLogo 16
#define IR_Menu 17
#define IR_A 18
#define IR_B 19
#define IR_Play 20
#define IR_X 21
//


//Low level functions:
extern int IRTransitionCount;
extern unsigned char IRBytes[20];
extern char IRActive;
extern volatile char IRReceiving;//note: IRReceiving turned off if IsIRDone() in regular or auto NavigationHandler() and causes ReadSideSensors() to repeat
//
extern void ModulateIR(unsigned int Frequency, unsigned int OnTime); //ModulateIR(38000,6) seems to produce best square wave for 38kHz.
extern void PlayChirpIR(unsigned int Frequency, unsigned int OnTime); //wrapper for ModulateIR, bkwd compatibility. use ModulateIR() instead of PlayChirpIR going forward
//
extern void IRHandler(void);
//
// ***************************************************
// end IR
// ***************************************************

// ***************************************************
// Light & Edge Sensing
// ***************************************************
//
#define LIGHT_SENSOR_STABLIZATION_TIME 200  //us
extern void ReadSideSensors(void);//single relative & ambient reading from each of the 3 side sensors (pauses and/or repeats if IRReceiving)
extern int RightLightLevel,LeftLightLevel,RearLightLevel;
extern int RearAmbientLightLevel,RightAmbientLightLevel,LeftAmbientLightLevel;
//
extern void ReadEdgeLightSensors(char Averages);//averages 8 reads
extern int RightEdgeLightLevel,LeftEdgeLightLevel,RearEdgeLightLevel;
//
extern void ResetLookAtEdge(void);//reset edge runnning average
extern void LookAtEdge(void);//take readings and add to running average (first call ResetLookAtEdge() once)
extern int LeftEdgeSensorAverage,RightEdgeSensorAverage,RearEdgeSensorAverage;
extern int LeftEdgeSensorValue,RightEdgeSensorValue,RearEdgeSensorValue;

//
//
//Stuff used in LookForEdge():
//
//Comment out if you want to disable the "check-twice" feature
//#define CHECK_EDGE_TWICE
//
//Normally, we compare readings to a running average to account for surfaces that have uneven reflectivity.
//These constants set absolute limits before deciding a reading is too light or dark.
//The dark value for the rear sensor is different since some light always reflects off of the coaster.
//Define DARK_MIN and DARK_MIN_REAR to be 0 and LIGHT_MAX to be 2000 if you want to disable this feature.
#define DARK_MIN 15
#define DARK_MIN_REAR 40
#define LIGHT_MAX 1000
//
//These functions & constants check for CHANGE in reading, and are proportional to the value of the average.
//Make the dark constants 0 and the light constants 10 in order to disable this feature.
//*Functions for fast multiplication by floating point number:
#define MultiplyBy7over8(val) ((val)-((val)>>3)) //"val-(val>>3)"  -> "val*7/8" (.75)
#define MultiplyBy6over8(val) ((val)-((val)>>2)) //"val-(val>>2)"  -> "val*6/8" (.875)
#define MultiplyBy9over8(val) ((val)+((val)>>3)) //"val+(val>>3)"  -> "val*9/8" (1.125 or 1 1/8)
#define MultiplyBy10over8(val) ((val)+((val)>>2)) //"val+(val>>2)"  -> "val*10/8" (1.25 or 1 2/8)
//*Constants for regular floating point multiplication:
#define DARK_EDGE_MULT_1 .8//.875
#define DARK_EDGE_MULT_2 .8//.75
#define LIGHT_EDGE_MULT_1 1.25//1.125
#define LIGHT_EDGE_MULT_2 1.25//1.25
//Un-comment one of either "Fast" or "Easy to read and adjust" sections below:
////If CHECK_EDGE_TWICE is not defined, then DarkEdgeMult2 and LightEdgeMult2 are not defined
//*Fast
/*#define DarkEdgeMult1(val) MultiplyBy9over8(val)
#define DarkEdgeMult2(val) MultiplyBy10over8(val)
#define LightEdgeMult1(val) MultiplyBy7over8(val)
#define LightEdgeMult2(val) MultiplyBy6over8(val) */
//*Easy to read and adjust
#define DarkEdgeMult1(val) (((float)(val))*DARK_EDGE_MULT_1)
#define DarkEdgeMult2(val) (((float)(val))*DARK_EDGE_MULT_2)
#define LightEdgeMult1(val) (((float)(val))*LIGHT_EDGE_MULT_1)
#define LightEdgeMult2(val) (((float)(val))*LIGHT_EDGE_MULT_2)
//
extern char LookForEdge(void);
#define RIGHT_DARK 0x01
#define RIGHT_BRIGHT 0x02
#define REAR_DARK 0x04
#define REAR_BRIGHT 0x08
#define LEFT_DARK 0x10
#define LEFT_BRIGHT 0x20
#define FrontEdgeDetected(edge) ((edge)&(RIGHT_DARK | RIGHT_BRIGHT | LEFT_DARK | LEFT_BRIGHT))
#define BackEdgeDetected(edge) ((edge)&(REAR_DARK | REAR_BRIGHT))
#define BrightDetected(edge) ((edge)&(RIGHT_BRIGHT | REAR_BRIGHT | LEFT_BRIGHT))
#define DarkDetected(edge) ((edge)&(RIGHT_DARK | REAR_DARK | LEFT_DARK))
#define RightDetected(edge) ((edge)&(RIGHT_BRIGHT | RIGHT_DARK))
#define LeftDetected(edge) ((edge)&(LEFT_BRIGHT | LEFT_DARK))
// LookForEdge() calls LookAtEdge(), and uses running average to detect dark tape/edges or white tape.
// dark edges are sensed as the edge sensor moves off the edge of a surface, as the surface is
// no longer present to reflect light to the sensor. dark edges are also sensed when driving
// on a light surface (like white paper) when coming in contact with a black line/tape/etc.
// bright/white edges are detected when moving the sensor from a relatively dark surface to
// a brighter surface. For example, encountering a piece of white paper when driving on a
// dark colored desk surface.
// Output bit num: 0: right dark edge (0x01)(0b000001)  1: right bright edge (0x02)(0b000010)   
//                 2: rear dark edge (0x04)(0b000100)   3: rear bright edge (0x08)(0b001000)  
//                 4: left dark edge (0x10)(0b010000)   5: left bright edge (0x20)(0b100000)
//
// ResetLookAtEdge();//call once in beginning
// while(1){edge=LookForEdge();if(edge&0x04){MoveForwardFast();}}//call LookForEdge() periodically
//
// Will not get stuck in on mode, but this means that if hovering over a dark edge or bright edge, it 
//  eventually "gets used' to it and trigger will go off.
// If not at least few milliseconds pause between calls, LookForEdge() might be less likely 
//  to detect an edge.
//Example for waiting till off of dark edge (or bright edge):
// If a left edge was encountered, you could store the current value of
//  LeftEdgeSensorAverage, then keep calling LookForEdge() (or just LookAtEdge()) until
//  LeftEdgeSensorValue is greater than the stored average value.
//



//
//Low level:
//
//For LookAtEdge()
extern int LeftEdgeArray[8];
extern int RightEdgeArray[8];
extern char EdgeArrayPos;
//
extern void EdgeLightsOn(void);
extern void EdgeLightsOff(void);
//
extern void SwitchAmbientToEdge(void);
extern void SwitchEdgeToAmbient(void);
//
extern int ReadLeftLightSensor(void);
extern int ReadRightLightSensor(void);
extern int ReadBackLightSensor(void);
//
//
// ***************************************************
// end Light & Edge Sensing
// ***************************************************


// ***************************************************
// EEPROM Functions
// ***************************************************

//Copied from http://playground.arduino.cc/Code/EEPROMWriteAnything  Visit page for use instructions.
#include <EEPROM.h>
#include <Arduino.h>  // for type definitions

template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          EEPROM.write(ee++, *p++);
    return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          *p++ = EEPROM.read(ee++);
    return i;
}

// ***************************************************
// end EEPROM Functions
// ***************************************************



// ***************************************************
// Recorded Data
// ***************************************************
  //Ver. 1.0, Dustin Soodak
  //
  //Edit this section for each use:
  typedef struct RecordedDataStruct{uint32_t ms; int degr;};//can edit number, names, and data types
  #define RECORDED_DATA_ARRAY_LENGTH 1 //set to 1 if not using
  #define RecordedDataPrintRow() do{Serial.print(Data.ms,DEC);Serial.print("\t");Serial.print(Data.degr,DEC);}while(0)
  //
  
  //Don't need to edit this section:
  extern RecordedDataStruct Data;
  extern RecordedDataStruct RecordedDataArray[RECORDED_DATA_ARRAY_LENGTH];
  extern unsigned char RecordedDataLength;
  extern unsigned char RecordedDataPosition;
  extern int RecordedDataN;
  extern uint32_t RecordedDataStart,RecordedDataPrev;
  extern uint16_t RecordedDataMinDelay;
  #define RecordedDataReset(uSDelay) \
    do{for(RecordedDataN=0;RecordedDataN<sizeof(RecordedDataArray);RecordedDataN++)  \
      ((char*)RecordedDataArray)[RecordedDataN]=0;  \
    RecordedDataPosition=0;RecordedDataLength=0;RecordedDataMinDelay=uSDelay;RecordedDataStart=micros();RecordedDataPrev=RecordedDataStart-RecordedDataMinDelay;}while(0)
   #define RecordedDataRefresh() \ 
    do{if(RecordedDataMinDelay==0 || micros()-RecordedDataPrev>=RecordedDataMinDelay){  \
           RecordedDataArray[RecordedDataPosition]=Data;  \
           if(RecordedDataPosition<RECORDED_DATA_ARRAY_LENGTH-1) {RecordedDataPosition++;} else {RecordedDataPosition=0;} \
           if(RecordedDataLength<RECORDED_DATA_ARRAY_LENGTH){RecordedDataLength++;}  \
           RecordedDataPrev+=RecordedDataMinDelay;}  \
     }while(0)
  
  #define RecordedDataFull() (RecordedDataLength==RECORDED_DATA_ARRAY_LENGTH)
  #define RecordedDataTime() (micros()-RecordedDataStart)
  #define RecordedDataPrint() do{for(RecordedDataN=0;RecordedDataN<RecordedDataLength;RecordedDataN++){  \
    Data=RecordedDataArray[(RecordedDataPosition+RecordedDataN)%RECORDED_DATA_ARRAY_LENGTH];RecordedDataPrintRow();Serial.println(); \
    }}while(0)
  //
  
  //Eample:  (tip:  TAB indents selected lines, SHIFT-TAB de-indents, CTRL-/ adds or removes line comments)
  ////Edited header file code:
  //typedef struct RecordedDataStruct{int t; int x;};
  //#define RECORDED_DATA_ARRAY_LENGTH 10
  //#define RecordedDataPrintRow() do{Serial.print(Data.t);Serial.print("_");Serial.print(Data.x);}while(0)
  ////In setup() or loop():
  //  Serial.println("manual delay:");
  //  RecordedDataReset(0);i=0;
  //  while(!RecordedDataFull()){
  //    Data.t=RecordedDataTime();Data.x=i;i++;//set fields of Data
  //    RecordedDataRefresh();//add Data to data array
  //    delay(1);//pause 1ms
  //  }
  //  RecordedDataPrint();
  //  Serial.println("automatic delay:");
  //  RecordedDataReset(1000);i=0;
  //  while(!RecordedDataFull()){
  //    Data.t=RecordedDataTime();Data.x=i;i++;//set fields of Data
  //    RecordedDataRefresh();//add Data to data array
  //  }
  //  RecordedDataPrint();
  
    
// ***************************************************
// end Recorded Data
// ***************************************************


// ***************************************************
// end Ringo Hardware
// ***************************************************


#endif
